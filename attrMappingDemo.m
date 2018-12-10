close all;
clear all;
clc
data_list = {...
    'site1';
    'site2';
    'site3';
    'site4';
    'site1_v';
    'site2_v';
    'site3_v';
    'site4_v';
    %     'run3_v';
    %     'Pandey';
    'kitti0002';
    'kitti0093';
    'kitti0104';
    'kitti0106';
    %
    'I475N_140903_h';
    'I475N_140903_v';
    };

% sm_list = {
%      'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%      'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%      'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
%      'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
%     };
sm_list = {'int-ref'};

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

% err_list = {...
%     'heading';
%     };

sm_type = 'NMI';

step_num = 30;

for di=1
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    nbins = round(256/para.bin_fraction);
    sm_num = length(para.sm_list);
    frame_num = length(img_list);
    tmp_x0 = para.x0;
    for img_idx = 1:frame_num
        image = img_list{img_idx};
        frame_no = image.frame_no
        R_w2v = makeRotationMatrix(image.prh);
        inv_R_w2v = inv(R_w2v);
        %
        %         dir_frame = ['frame_' num2str(frame_no) '\'];
        %         mkdir([para.dir_output dir_frame]);
        
        %     h = fspecial('gaussian',11*[1,1],6);
        % image - intensity
        inten_img = rgb2gray(image.image);
        inten_img_eq = adapthisteq(inten_img);
        % image - illumination inv
        tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        y_img = tmp_img(:,:,1);
        %     cb_img = tmp_img(:,:,2);
        %     cr_img = tmp_img(:,:,3);
        tmp_img = rgb2hsv(double(image.image));
        h_img = tmp_img(:,:,1);
        %     s_img = tmp_img(:,:,2);
        %     v_img = tmp_img(:,:,3);
        % image - gradient
        % [~, gd_img_int] = imgradient(inten_img);
        % [~, gd_img_inteq] = imgradient(inten_img_eq);
        [gm_img_hue, ~] = imgradient(h_img);
        %
        %     edge_img_int = edge(inten_img, 'Canny', [0.05, 0.3]);
        
        [tmp_x0, t, R, inv_R, ~] = addError(para.err_list{1}, tmp_x0, 1+step_num, step_num);
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        cur_points = points(points_id, :);
        
        figure;
        imagesc(image.image);
        hold on
        scatter(u, v, 3*ones(1,length(u)), cur_points(:, 6), 'filled');
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        file_name = sprintf('projImage_%05d.jpg', frame_no);
        saveas(gcf, [para.dir_output, file_name]);
        
        % point cloud - reflectivity
        ref_pcd_arr = double(cur_points(:,5)); % refc
        %             ref_pcd_arr_upd = log(ref_pcd_arr);
        %             ref_pcd_arr_upd(ref_pcd_arr==0) = 0;
        %             ref_pcd_arr_eq = histeq(ref_pcd_arr);
        %             ref_pcd_arr_eq = double(points(points_id,6));
        % point cloud - normal
        nx_pcd_arr = double(cur_points(:,7));
        ny_pcd_arr = double(cur_points(:,8));
        nz_pcd_arr = double(cur_points(:,9));
        % convert normal from wc to vc
        norm_VC = inv_R_w2v * [nx_pcd_arr, ny_pcd_arr, nz_pcd_arr]';
        nx_pcd_arr = norm_VC(1, :)';
        ny_pcd_arr = norm_VC(2, :)';
        nz_pcd_arr = norm_VC(3, :)';
        % point cloud - curvature
        curv_pcd_arr = double(cur_points(:,10)); % cur
        curv_pcd_arr = (curv_pcd_arr - min(curv_pcd_arr)) / (max(curv_pcd_arr) - min(curv_pcd_arr));
        gm_ref_arr = double(cur_points(:,11));
        gm_ref_arr = (gm_ref_arr - min(gm_ref_arr)) / (max(gm_ref_arr) - min(gm_ref_arr));
        gm_rng_arr = double(cur_points(:,12));
        gm_rng_arr = (gm_rng_arr - min(gm_rng_arr)) / (max(gm_rng_arr) - min(gm_rng_arr));
        %
        curv_comb_arr = curv_pcd_arr + gm_rng_arr;
        grad_comb_arr = curv_comb_arr + gm_ref_arr;
        
        for sn = 1 : sm_num
            % compute sm for each pair of attributes
            switch para.sm_list{sn}
                %
                case 'int-ref'
                    %                     img_attr = im2double(inten_img);
                    %                     pcd_attr = double(ref_pcd_arr);
                    img_attr = inten_img;
                    pcd_attr = ref_pcd_arr;
                case 'int_{hist}-ref'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(ref_pcd_arr);
                case 'int_{shadow}-ref'
                    img_attr = im2double(y_img);
                    pcd_attr = double(ref_pcd_arr);
                case 'hue-ref'
                    img_attr = im2double(h_img);
                    pcd_attr = double(ref_pcd_arr);
                    %
                case 'int-norm_x'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nx_pcd_arr);
                case 'int-norm_y'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(ny_pcd_arr);
                case 'int-norm_z'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nz_pcd_arr);
                    %
                case 'int_{hist}-norm_x'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(nx_pcd_arr);
                case 'int_{hist}-norm_y'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(ny_pcd_arr);
                case 'int_{hist}-norm_z'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(nz_pcd_arr);
                    %
                case 'grad^{hue}_{mag}-curv'
                    img_attr = im2double(gm_img_hue);
                    pcd_attr = double(curv_comb_arr);
                case 'grad^{hue}_{mag}-grad^{ref}_{mag}'
                    img_attr = im2double(gm_img_hue);
                    pcd_attr = double(gm_ref_arr);
                case 'grad^{hue}_{mag}-grad^{comb}_{mag}'
                    img_attr = im2double(gm_img_hue);
                    pcd_attr = double(grad_comb_arr);
            end
            
            img_attr = img_attr(:)';
            img_attr = img_attr(v + (u-1)*para.cam.height)';
            %                     valid_idx = points_idx &((u>=1 & u<=img_w) & (v>=1 & v<=img_h));
            
            %% get mapping between 2D & 3D attributes
            %             [MI_hist, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
            [MI_hist, MI_prob] = get_histogram_no_norm(img_attr, [0 255], pcd_attr, [0 127], nbins);
            
            % visulization
            figure;
            imagesc(MI_hist.hist_joint);
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 12])
            file_name = sprintf('jointHist_%05d.jpg', frame_no);
            saveas(gcf, [para.dir_output, file_name]);
            close all;
            
            %
            %                 tic
            %                 [~, MI_prob] = get_histogram_gpu(img_attr, pcd_attr, nbins);
            
            %                 [sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob, nbins);
            %             cost_tmp = MI_cpp(img_attr, pcd_attr, length(img_attr), nbins);
            %             if strcmp(para.SM_type, 'NMI')% NMI
            %                 sm_arr(iter_no, sn) = cost_tmp(1);
            %             else% MI
            %                 sm_arr(iter_no, sn) = cost_tmp(2);
            %             end
            %                 toc
            %                 sm_arr(iter_no, sn) = LSQMIregression(img_attr', pcd_attr');
        end
        %     computeSMTrend(img_list, points, para);
        % para.x0 = gradient_descent_search_grid (para.x0, img_list, points, para);
        % mi_cost_2d(para.x0, img_list, points, para,'test_rst');
    end
end