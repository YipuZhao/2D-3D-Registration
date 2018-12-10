function [dist_buf] = projectionErrorQuantify(img_list, points, para, approach_list)

show_img = true

% t, inv_R: lidar to cam
% R = makeRotationMatrix(para.x0(4:6));
% inv_R = inv(R);
% t = para.x0(1:3)';

sm_num = length(para.sm_list);
frame_num = length(img_list);
nbins = round(256/para.bin_fraction);
grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
grid_x(end) = para.ROI_width(2)+1;
grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
grid_y(end) = para.ROI_height(2)+1;

step_num = 20;

bias_num = 2;

% para.x0(6) = para.x0(6) + deg2rad(0.1) * (5);
% tmp_x0 = para.x0;
% sm_min = 0.8;
% sm_max = 1.5;
sm_min = 0;
sm_max = 0.15;

% (img_idx, iter_no, sn, i, j)
sm_grid = zeros(2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
sm_arr = zeros(2*step_num+1, sm_num);

% dist_buf = -1 * ones(3, length(para.err_list), 11, 3 * 8);
dist_buf = -1 * ones(2, length(para.err_list), 1+2*bias_num, 8);
% st_idx = ones(3, length(para.err_list), 11);

% yuanfang_output = '..\..\data\GT_reorg\segmentation\site1_v\';
err_step = [deg2rad(0.1),deg2rad(0.1),deg2rad(0.1),0.025,0.025,0.025];

img_count = 1;
for img_idx = 1:frame_num
    image = img_list{img_idx};
    frame_no = image.frame_no
    
%     if frame_no ~= 4310 && frame_no ~= 4940 && frame_no ~= 4980
%         continue ;
%     end
    if frame_no ~= 60 && frame_no ~= 100 && frame_no ~= 160
        continue ;
    end
        
    R_w2v = makeRotationMatrix(image.prh);
    inv_R_w2v = inv(R_w2v);
    %
    dir_frame = ['frame_' num2str(frame_no) '\'];
    %     mkdir([para.dir_output dir_frame]);
    
    [~, t, R, inv_R, err_val] = addError(para.err_list{1}, para.x0, 1+step_num, step_num);
    [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
    [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
    cur_points = points(points_id, :);
    
    % visualize current image and range
    figure;
    imagesc(image.image);
    hold on;
    scatter(u,v,ones(1,length(u)), cur_points(:, 5));
    
%     [img_x, img_y] = ginput(1)
    [img_x, img_y] = ginput(8)
    % find corresponding 3D points
    ref_points=[];
    for i=1:length(img_x)
        [min_dist, min_idx]=min(abs(u-img_x(i))+abs(v-img_y(i)));
        ref_points = [ref_points;u(min_idx), v(min_idx), cur_points(min_idx, :)]
    end
    scatter(ref_points(:,1), ref_points(:,2), 100, '+g');
    
    %     iter_no = 1;
    for eno = 1:length(para.err_list)
        err_fname = ['error_' para.err_list{eno} '\']
        for iter_no = -step_num:step_num
%             if iter_no == 0
%                 continue ;
%             end
            for ano=1:length(approach_list)
                reg_fname = sprintf('%s_%s_%d.mat', approach_list{ano}, para.err_list{eno}, iter_no + step_num + 1);
                load([para.dir_output dir_frame err_fname reg_fname]);
                % reg_fname = sprintf('err%s_%.04f_whole_%d.mat', para.err_list{eno}, iter_no * err_step(eno), approach_list{ano});
                % para_bak = para;
                % load([yuanfang_output err_fname reg_fname]);
                % tmp_x0 = para.x0;
                % para = para_bak;
                if ano == 1 || ano == 2
                    tmp_x0 = mi_x0
                else
                    tmp_x0 = nmi_x0
                end
                %% add error to registration parameters
                [~, t, R, inv_R, err_val] = addError(para.err_list{1}, tmp_x0, 1+step_num, step_num);
                %         tic
                %[u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
                [ref_u_tmp, ref_v_tmp, points_id] = imageProjection_upd(ref_points(:, 3:end), image, para, t, R, inv_R);
                %%
                ref_gt = ref_points(points_id, 1:2);
                for i=1:length(points_id)
                    dist_tmp(i) = pdist2([ref_gt(i,1), ref_gt(i,2)], [ref_u_tmp(i), ref_v_tmp(i)]);
                end
                dist_buf(ano, eno, iter_no+step_num+1, 8 * (img_count-1) + 1 : 8 * img_count) = dist_tmp;
                
            end
        end
    end
    
    img_count = img_count + 1;
end
