function [mi_cost_avg] = weighted_mi_cost_compute(x0, img_list, points, point_voxel_arr, voxel_weight_arr, para, w)
%%

global sm_num

% show_img = ~isempty(file_head);
mi_cost_allframe = [];

R = makeRotationMatrix(x0(4:6));
inv_R = inv(R);
t = x0(1:3)';
nbins = round(256/para.bin_fraction);

frame_num = length(img_list);
for img_idx = 1:frame_num
    %
    %     disp('collect attr---------');
    %     tic
    
    image = img_list{img_idx};
    R_w2v = makeRotationMatrix(image.prh);
    inv_R_w2v = inv(R_w2v);
    
    [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
    voxel_idx = point_voxel_arr(2, points_id);
    weight_tmp = voxel_weight_arr(voxel_idx);
    points_tmp = points(points_id, :);
    
    ref_pcd_arr = double(points_tmp(:, 5)); % refc
    nx_pcd_arr = double(points_tmp(:,7));
    ny_pcd_arr = double(points_tmp(:,8));
    nz_pcd_arr = double(points_tmp(:,9));
    % convert normal from wc to vc
    norm_VC = inv_R_w2v * [nx_pcd_arr, ny_pcd_arr, nz_pcd_arr]';
    nx_pcd_arr = norm_VC(1, :)';
    ny_pcd_arr = norm_VC(2, :)';
    nz_pcd_arr = norm_VC(3, :)';
    % point cloud - curvature
    curv_pcd_arr = double(points_tmp(:,10)); % cur
    curv_pcd_arr = (curv_pcd_arr - min(curv_pcd_arr)) / (max(curv_pcd_arr) - min(curv_pcd_arr));
    gm_ref_arr = double(points_tmp(:,11));
    gm_ref_arr = (gm_ref_arr - min(gm_ref_arr)) / (max(gm_ref_arr) - min(gm_ref_arr));
    gm_rng_arr = double(points_tmp(:,12));
    gm_rng_arr = (gm_rng_arr - min(gm_rng_arr)) / (max(gm_rng_arr) - min(gm_rng_arr));
    %
    curv_comb_arr = curv_pcd_arr + gm_rng_arr;
    grad_comb_arr = curv_comb_arr + gm_ref_arr;
    
    %     toc
    
    %         tic
    sm_arr = zeros(1, sm_num);
    for sn = 1 : sm_num
        %         disp('retrieve---------');
        %         tic
        % compute sm for each pair of attributes
        switch para.sm_list{sn}
            %
            case 'int-ref'
                img_attr = (image.img_attribute{1});
                pcd_attr = double(ref_pcd_arr);
            case 'int_{hist}-ref'
                img_attr = (image.img_attribute{2});
                pcd_attr = double(ref_pcd_arr);
            case 'int_{shadow}-ref'
                img_attr = (image.img_attribute{3});
                pcd_attr = double(ref_pcd_arr);
            case 'hue-ref'
                img_attr = (image.img_attribute{4});
                pcd_attr = double(ref_pcd_arr);
                %
            case 'int-norm_x'
                img_attr = (image.img_attribute{1});
                pcd_attr = double(nx_pcd_arr);
            case 'int-norm_y'
                img_attr = (image.img_attribute{1});
                pcd_attr = double(ny_pcd_arr);
            case 'int-norm_z'
                img_attr = (image.img_attribute{1});
                pcd_attr = double(nz_pcd_arr);
                %
            case 'int_{hist}-norm_x'
                img_attr = image.img_attribute{2};
                pcd_attr = double(nx_pcd_arr);
            case 'int_{hist}-norm_y'
                img_attr = image.img_attribute{2};
                pcd_attr = double(ny_pcd_arr);
            case 'int_{hist}-norm_z'
                img_attr = image.img_attribute{2};
                pcd_attr = double(nz_pcd_arr);
                %
            case 'grad^{hue}_{mag}-curv'
                img_attr = image.img_attribute{5};
                pcd_attr = double(curv_comb_arr);
            case 'grad^{hue}_{mag}-grad^{ref}_{mag}'
                img_attr = image.img_attribute{5};
                pcd_attr = double(gm_ref_arr);
            case 'grad^{hue}_{mag}-grad^{comb}_{mag}'
                img_attr = image.img_attribute{5};
                pcd_attr = double(grad_comb_arr);
        end
        
        img_attr = img_attr(:)';
        img_attr = img_attr(v + (u-1)*para.cam.height)';
        %         toc
        %     valid_idx = points_idx &((u>=1 & u<=img_w) & (v>=1 & v<=img_h));
        %
        %         disp('compute---------');
        %         tic
        [~, MI_prob] = get_histogram_weighted(img_attr, pcd_attr, weight_tmp, nbins);
        % NMI
        if strcmp(para.SM_type, 'NMI')
            [sm_arr(sn), ~, ~, ~] = get_NMI(MI_prob);
            % MI
        else
            [~, sm_arr(sn), ~, ~] = get_NMI(MI_prob);
        end
%         cost_tmp = MI_cpp(img_attr, pcd_attr, length(img_attr), nbins);
%         if strcmp(para.SM_type, 'NMI')% NMI
%             sm_arr(sn) = cost_tmp(1);
%         else% MI
%             sm_arr(sn) = cost_tmp(2);
%         end
        %         toc
    end
    %
    %         toc
    if length(points_id)  < para.point_thres
        mi_cost = 0;
    else
        mi_cost = w{img_idx} * sm_arr';
    end
    
    %     mi_cost_oneframe = cell2mat(mi_cost);
    %     mi_cost = mi_cost([mi_cost.mi]~=0 & [mi_cost.num]>=para.GRID_THRD);
    mi_cost_allframe = [mi_cost_allframe, mi_cost];
    %     if show_img
    %         mi_cost = mi_cost([mi_cost.mi_cost.mi]~=0 & [mi_cost.num]>=para.GRID_THRD);
    %         save_image_seg(file_head, para, image.frame_no, u, v, image_toshow, points_pts, points_seg, mi_cost);
    %     end
end

mi_cost_avg = mean(mi_cost_allframe);
