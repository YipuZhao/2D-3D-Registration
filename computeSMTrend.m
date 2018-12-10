function [sm_grid] = computeSMTrend(img_list, points, para)

show_img = true

% t, inv_R: lidar to cam
% R = makeRotationMatrix(para.x0(4:6));
% inv_R = inv(R);
% t = para.x0(1:3)';

sm_num = length(para.sm_list);
frame_num = length(img_list);
nbins = round(256/para.bin_fraction);
% grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
% grid_x(end) = para.ROI_width(2)+1;
% grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
% grid_y(end) = para.ROI_height(2)+1;

step_num = 20;

% para.x0(6) = para.x0(6) + deg2rad(0.1) * (-3);
tmp_x0 = para.x0;
% sm_min = 0.8;
% sm_max = 1.5;
sm_min = 0;
sm_max = 0.15;

% (img_idx, iter_no, sn, i, j)
% sm_grid = zeros(2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
sm_arr = zeros(2*step_num+1, sm_num);

for img_idx = 1:frame_num
    image = img_list{img_idx};
    frame_no = image.frame_no
    R_w2v = makeRotationMatrix(image.prh);
    inv_R_w2v = inv(R_w2v);
    %
    dir_frame = ['frame_' num2str(frame_no) '\'];
    mkdir([para.dir_output dir_frame]);
    
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
    % for kitti data
    if strcmp(para.camera_type, 'kitti')
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        % for GT data
    else
        [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points);
    end
    cur_points = points(points_id, :);
    
    %     iter_no = 1;
    for eno = 1:length(para.err_list)
        tic
        err_frame = ['error_' para.err_list{eno} '\'];
        mkdir([para.dir_output dir_frame err_frame]);
        for iter_no = 1:1+2*step_num
            %% add error to registration parameters
            [~, t, R, inv_R, ~] = addError(para.err_list{eno}, tmp_x0, iter_no, step_num);
            %         tic
            %[u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
            [u, v, points_id] = imageProjection_upd(cur_points, image, para, t, R, inv_R);
            %             [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
            %         toc
            
            %         tic
            % point cloud - reflectivity
            ref_pcd_arr = double(cur_points(points_id,5)); % refc
            %             ref_pcd_arr_upd = log(ref_pcd_arr);
            %             ref_pcd_arr_upd(ref_pcd_arr==0) = 0;
            %             ref_pcd_arr_eq = histeq(ref_pcd_arr);
            %             ref_pcd_arr_eq = double(points(points_id,6));
            % point cloud - normal
            nx_pcd_arr = double(cur_points(points_id,7));
            ny_pcd_arr = double(cur_points(points_id,8));
            nz_pcd_arr = double(cur_points(points_id,9));
            % convert normal from wc to vc
            norm_VC = inv_R_w2v * [nx_pcd_arr, ny_pcd_arr, nz_pcd_arr]';
            nx_pcd_arr = norm_VC(1, :)';
            ny_pcd_arr = norm_VC(2, :)';
            nz_pcd_arr = norm_VC(3, :)';
            % point cloud - curvature
            curv_pcd_arr = double(cur_points(points_id,10)); % cur
            curv_pcd_arr = (curv_pcd_arr - min(curv_pcd_arr)) / (max(curv_pcd_arr) - min(curv_pcd_arr));
            gm_ref_arr = double(cur_points(points_id,11));
            gm_ref_arr = (gm_ref_arr - min(gm_ref_arr)) / (max(gm_ref_arr) - min(gm_ref_arr));
            gm_rng_arr = double(cur_points(points_id,12));
            gm_rng_arr = (gm_rng_arr - min(gm_rng_arr)) / (max(gm_rng_arr) - min(gm_rng_arr));
            %
            curv_comb_arr = curv_pcd_arr + gm_rng_arr;
            grad_comb_arr = curv_comb_arr + gm_ref_arr;
            %             curv_pcd_arr_eq = histeq(curv_pcd_arr);
            % point cloud - edge
            %             edge_pcd_arr = double(cur_points(points_id,11));
            % combine
            %         max_curv = max(curv_pcd_arr);
            %         curv_pcd_arr = curv_pcd_arr + edge_pcd_arr * max_curv;
            %             max_curv = max(curv_pcd_arr);
            %             curv_pcd_arr_comb = curv_pcd_arr + edge_pcd_arr * max_curv;
            %         toc
            
            %         tic
            %%
            %             grid_point_num = zeros(para.grid_row_num, para.grid_col_num);
            %             grid_point_area = zeros(para.grid_row_num, para.grid_col_num);
            %             entropy_joint = zeros(nbins, nbins, para.grid_row_num, para.grid_col_num);
            %             tic
            for sn = 1 : sm_num
                % compute sm for each pair of attributes
                switch para.sm_list{sn}
                    %
                    case 'int-ref'
                        img_attr = im2double(inten_img);
                        pcd_attr = double(ref_pcd_arr);
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
                
                %
                %                 tic
                %                 [~, MI_prob] = get_histogram_gpu(img_attr, pcd_attr, nbins);
                % for kitti data
                if strcmp(para.camera_type, 'kitti')
                    [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
                    [sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob, nbins);
                    % for GT data
                else
                    cost_tmp = MI_cpp(img_attr, pcd_attr, length(img_attr), nbins);
                    if strcmp(para.SM_type, 'NMI')% NMI
                        sm_arr(iter_no, sn) = cost_tmp(1);
                    else% MI
                        sm_arr(iter_no, sn) = cost_tmp(2);
                    end
                end
                %                 toc
                %                 sm_arr(iter_no, sn) = LSQMIregression(img_attr', pcd_attr');
            end
            %             toc
            %         toc
            if show_img %&& err_heading == 0
                if iter_no == 1 + step_num && eno == 1
                    saveFeatureImage(image, para, u, v, [dir_frame], ...
                        inten_img, inten_img_eq, y_img, h_img, gm_img_hue, ...
                        ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_comb_arr, gm_ref_arr, grad_comb_arr);
                    saveProjectionImage(image, para, u, v, ...
                        [dir_frame], ...
                        inten_img, ref_pcd_arr);
                end
            end
        end
        toc
        %%
        % img_idx, iter_no, sn, i, j
        cur_sm_arr(:, :) = sm_arr(:, :);
        %         cur_sm_grid(:, :, :, :) = sm_grid(:, :, :, :);
        file_name = sprintf('SM_frame_%04d_%s', frame_no, para.err_list{eno});
        %         save([para.dir_output, dir_frame, err_frame, file_name '.mat'], 'cur_sm_arr', 'cur_sm_grid');
        save([para.dir_output, dir_frame, err_frame, file_name '.mat'], 'cur_sm_arr');
        
        figure;
        min_val = min(cur_sm_arr(:, :), [], 1);
        plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
        hold on;
        plot(-step_num : step_num, cur_sm_arr(:, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1), 1), '-x');
        plot(-step_num : step_num, cur_sm_arr(:, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1), 1), '-.');
        %                 plot(-step_num : step_num, cur_sm_arr(:, 19:22), '-');
        %     ylim([sm_min sm_max]);
        legend(para.sm_list);
        %     set(gcf, 'PaperPositionMode', 'auto')
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        %     maximize(gcf)
        file_name = sprintf('Trend_frame_%04d_%s', frame_no, para.err_list{eno});
        saveas(gcf, [para.dir_output, dir_frame, err_frame, file_name]);
        %     export_fig( gcf, ...
        %         [para.dir_output, dir_frame, file_name], ...
        %         '-painters', '-jpg', '-r300' );
        close all;
    end
end

