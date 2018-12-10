clear all
close all
global err_num
global para
global w
global step_num
global sm_num

global reg_img_list
global reg_points
global point_voxel_arr
% global voxel_weight_arr
global weight_count

%%

data_list = {
    'site1';    'site2';    'site3';    'site4';
    'site1_v';    'site2_v';    'site3_v';    'site4_v';
    'kitti0002';    'kitti0093';    'kitti0104';    'kitti0106';
    'change_ref0_h'; 'change_ref0_v'; 'change_tilt0_h'; 'change_tilt0_v';
    'I475N_141112_h'; 'I475N_141112_v';
    'I475N_150130_h'; 'I475N_150130_v';
    'I475N_150529_h'; 'I475N_150529_v';
    'I475N_150701_h'; 'I475N_150701_v';
    'calib_150326_h'; 'calib_150326_v';
    };

% sm_list = {
%      'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%      'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%      'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
%      'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
%     };
sm_list = {
    'int-ref';
    };

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

sm_type = 'NMI';

skip_frame_num = 5;%15;
sm_min = 0;
sm_max = 0.01;
step_num = 20;
sm_num = length(sm_list);
err_num = 6;
% dir_cluster = ['C:\Users\yzhao347\Copy\cluster_test\'];
% dir_cluster = ['D:\\Copy\cluster_atl_v\'];
% load([dir_cluster 'cluster.mat']);
% load([dir_cluster 'data.mat']);

% para.x0 = [-0.4765748086480437,	-0.10868538971935367,	0.360704670619655,...
%     -0.1049723896187755,	-0.020188580624356477,	-0.3469560326690064
% ];
%
rno = 1;
reg_points = [];
% points_weight = [];
for di = 5
    % load para
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    nbins = round(256/para.bin_fraction);
    %     tmp_x0 = para.x0;
    % load data
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    frame_num = length(img_list);
    %
    % points_weight = ones(1, length(points));
    
    for img_idx = 1:20%1:skip_frame_num:frame_num
        image = img_list{img_idx};
        frame_no = image.frame_no
        dir_frame = ['frame_' num2str(frame_no) '\'];
        % estimate 2D attributes
        image.img_attribute{1} = im2double(rgb2gray(image.image));
        image.img_attribute{2} = im2double(adapthisteq(image.img_attribute{1}));
        % image - illumination inv
        tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        image.img_attribute{3} = im2double(tmp_img(:,:,1));
        %     cb_img = tmp_img(:,:,2);
        %     cr_img = tmp_img(:,:,3);
        tmp_img = rgb2hsv(double(image.image));
        image.img_attribute{4} = im2double(tmp_img(:,:,1));
        %     s_img = tmp_img(:,:,2);
        %     v_img = tmp_img(:,:,3);
        % image - gradient
        % [~, gd_img_int] = imgradient(inten_img);
        % [~, gd_img_inteq] = imgradient(inten_img_eq);
        [gm_img, gd_img] = imgradient(image.img_attribute{4} );
        image.img_attribute{5} = im2double(gm_img);
        %         image.img_attribute{6} = im2double(gm_img);
        %
        [~, t, R, inv_R, err_val] = addError(para.err_list{1}, para.x0, step_num+1, step_num);
        % get corresponding points
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        reg_img_list{rno} = image;
        reg_points = [reg_points; points(points_id, :)];
        % get scene label
        %                 if img_list{img_idx}.label == -1 || sm_num == 1
        %                     w{rno} = ones(1, sm_num);
        %                     w{rno} = w{rno} / sum(w{rno});
        %                 else
        %                     w{rno} = cluster{img_list{img_idx}.label}.w;
        %                 end
        w{rno} = ones(1, sm_num);
        % w{rno} = [0.380779354867005,0.334985625622913,0.00411299275633137,0.160203562012357,0.000474060560658990,1.03066049044463e-05,0.0733103561542996,3.48838244033863e-05,0.000969274365406947,0.0204123504227138,0.0177457287967988,0.00696115309997825,3.50912229108639e-07];
        % w{rno} = w{rno} / sum(w{rno});
        
        rno = rno + 1;
    end
    
    % point cloud voxelization
    [point_voxel_arr] = pointCloudVoxelize(reg_points, 5000);
    voxel_weight_arr = ones(1, max(point_voxel_arr(2,:)));
    weight_count = sum(voxel_weight_arr);
    
    %     voxel_weight_arr = fminsearchbnd(...
    %         @weightErrorEstimate, voxel_weight_arr, ...
    %         optimset('PlotFcns', @optimplotfval, 'MaxIter', 3000));
    voxel_weight_arr = fminsearchbnd(...
        @weightErrorEstimate, voxel_weight_arr, ...
        zeros(1, length(voxel_weight_arr)), 10 * ones(1, length(voxel_weight_arr)),...
        optimset('PlotFcns', @optimplotfval, 'MaxIter', 10000));
    %     points_weight = gradient_descent_weightOpt (para.x0, reg_img_list, reg_points, para, w);
    % nmi_x0 = gradient_descent_search_s2 (para.x0, reg_img_list, reg_points, para, w);
    
    weight_arr = ones(1, length(reg_points));
    for i=1 : max(point_voxel_arr(2,:))
        tmp_idx = find(point_voxel_arr(2,:) == i);
        point_idx = point_voxel_arr(1, tmp_idx);
        weight_arr(point_idx) = voxel_weight_arr(i);
    end
    
    for img_idx = 1:20
        image = reg_img_list{img_idx};
        frame_no = image.frame_no
        R_w2v = makeRotationMatrix(image.prh);
        inv_R_w2v = inv(R_w2v);
        %         frame_no = reg_img_list{img_idx}.frame_no
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
        
        [~, t, R, inv_R, err_val] = addError(para.err_list{1}, para.x0, step_num+1, step_num);
        % get corresponding points
        [u, v, points_id] = imageProjection_upd(reg_points, reg_img_list{img_idx}, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, reg_img_list{img_idx}, reg_points);
        
        figure;
        imagesc(reg_img_list{img_idx}.image);
        hold on;
        scatter(u, v, ones(1, length(points_id)), weight_arr(points_id));
        file_name = sprintf('Weight_frame_%d.jpg', frame_no);
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        saveas(gcf,[para.dir_output, file_name]);
        %%
        cur_points = reg_points(points_id, :);
        cur_weights = weight_arr(points_id);
        dir_frame = ['frame_' num2str(frame_no) '\'];
        for eno = 1:length(para.err_list)
            tic
            err_frame = ['error_' para.err_list{eno} '\'];
            mkdir([para.dir_output dir_frame err_frame]);
            for iter_no = 1:1+2*step_num
                %% add error to registration parameters
                [~, t, R, inv_R, ~] = addError(para.err_list{eno}, para.x0, iter_no, step_num);
                %         tic
                %[u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
                [u, v, points_id] = imageProjection_upd(cur_points, image, para, t, R, inv_R);
                %             [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                %         toc
                
                %         tic
                weight_pcd_arr = cur_weights(points_id);
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
                    
                    % weighted mi
                    [~, MI_prob] = get_histogram_weighted(img_attr, pcd_attr, weight_pcd_arr, nbins);
                    % NMI
                    if strcmp(para.SM_type, 'NMI')
                        [weighted_sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob);
                        % MI
                    else
                        [~, weighted_sm_arr(iter_no, sn), ~, ~] = get_NMI(MI_prob);
                    end
                    [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
                    % NMI
                    if strcmp(para.SM_type, 'NMI')
                        [sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob);
                        % MI
                    else
                        [~, sm_arr(iter_no, sn), ~, ~] = get_NMI(MI_prob);
                    end
                    %                 tic
                    %                 [~, MI_prob] = get_histogram_gpu(img_attr, pcd_attr, nbins);
                    %                 [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
                    %                 [sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob, nbins);
                    %                 cost_tmp = MI_cpp(img_attr, pcd_attr, length(img_attr), nbins);
                    %                 if strcmp(para.SM_type, 'NMI')% NMI
                    %                     sm_arr(iter_no, sn) = cost_tmp(1);
                    %                 else% MI
                    %                     sm_arr(iter_no, sn) = cost_tmp(2);
                    %                 end
                    %                 toc
                    %                 sm_arr(iter_no, sn) = LSQMIregression(img_attr', pcd_attr');
                end
                %             toc
                %         toc
                %                 if show_img %&& err_heading == 0
                if iter_no == 1 + step_num && eno == 1
                    saveFeatureImage(image, para, u, v, [dir_frame], ...
                        inten_img, inten_img_eq, y_img, h_img, gm_img_hue, ...
                        ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_comb_arr, gm_ref_arr, grad_comb_arr);
                    saveProjectionImage(image, para, u, v, ...
                        [dir_frame], ...
                        inten_img, ref_pcd_arr);
                end
                %                 end
            end
            toc
            %%
            % img_idx, iter_no, sn, i, j
            %         cur_sm_grid(:, :, :, :) = sm_grid(:, :, :, :);
            file_name = sprintf('SM_frame_%04d_%s', frame_no, para.err_list{eno});
            %         save([para.dir_output, dir_frame, err_frame, file_name '.mat'], 'cur_sm_arr', 'cur_sm_grid');
            save([para.dir_output, dir_frame, err_frame, file_name '.mat'], 'sm_arr', 'weighted_sm_arr');
            
            figure;
            min_val = min(sm_arr(:, :), [], 1);
            plot(-step_num : step_num, sm_arr(:) - min_val, '-o');
            hold on;
            min_val = min(weighted_sm_arr(:, :), [], 1);
            plot(-step_num : step_num, weighted_sm_arr(:) - min_val, '-x');
            %                 plot(-step_num : step_num, cur_sm_arr(:, 19:22), '-');
            %     ylim([sm_min sm_max]);
            legend({'nmi';'weighted nmi'});
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
        % restore
        %         para.x0 = ori_x0;
        %         close all;
    end
end