close all;
clear all;
clc
data_list = {...
    'site1';
    'site2';
    'site3';
    'site4';
    %
    'site2_run2';
    'site3_run2';
    %
    'site2_run3';
    'site3_run3';
    %     'run3_v';
    %     'Pandey';
    'kitti0002';
%     'kitti0093';
    'kitti0104';
    'kitti0106';
    %
    'I475N_150529';
    'I475S_150529';
    'I475N_150326';
    'I475S_150326';
    };

% sm_list = {
%      'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%      'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%      'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
%      'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
%     };
sm_list = {
    'int-int';
    'int_{hist}-int_{hist}';
    'int_{shadow}-int_{shadow}';
    'hue-hue';
    };

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

step_num = 20;

di=2

para = loadPara(data_list{di}, sm_list, err_list, sm_type);
[points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
%     points(:, 13:16) = -1;
color_buf = cell(1, length(points));

nbins = round(256/para.bin_fraction);
sm_num = length(para.sm_list);
frame_num = length(img_list);
% tmp_x0 = para.x0;

frame_limit = frame_num

%% projection
% for img_idx = 1:frame_limit%frame_num
%     img_idx
%     
%     tic
%     disp('1. projecting to 2D image');
%     prior_image = img_list{img_idx};
%     [~, t, R, inv_R, ~] = addError(para.err_list{1}, para.x0, 1+step_num, step_num);
%     [u, v, points_id] = imageProjection_upd(points, prior_image, para, t, R, inv_R);
%     toc
%     
%     tic
%     disp('2. occlusion removal');
%     [u, v, points_id] = occludedPointsRemoval(u, v, points_id, prior_image, points);
%     toc
%     
%     % project rgb info from image to point cloud
%     tic
%     disp('3. histogram equalization');
%     prior_int_img = rgb2gray(prior_image.image);
%     prior_inteq_img = adapthisteq(prior_int_img);
%     prior_r_img = (prior_image.image(:,:,1));
%     prior_g_img = (prior_image.image(:,:,2));
%     prior_b_img = (prior_image.image(:,:,3));
% %     tmp_img = shadowremoval(im2double(prior_image.image), 'ycbcr');
% %     prior_y_img = tmp_img(:,:,1);
% %     tmp_img = rgb2hsv(double(prior_image.image));
% %     prior_h_img = tmp_img(:,:,1);
%     %
%     prior_int_arr = im2double(prior_int_img);
%     prior_inteq_arr = im2double(prior_inteq_img);
%     prior_r_arr = im2double(prior_r_img);
%     prior_g_arr = im2double(prior_g_img);
%     prior_b_arr = im2double(prior_b_img);
% %     prior_y_arr = im2double(prior_y_img);
% %     prior_h_arr = im2double(prior_h_img);
%     %
%     %         prior_int_arr = prior_int_arr(:)';
%     %         points(points_id, 13) = prior_int_arr(v + (u-1)*para.cam.height)';
%     %         prior_inteq_arr = prior_inteq_arr(:)';
%     %         points(points_id, 14) = prior_inteq_arr(v + (u-1)*para.cam.height)';
%     %         prior_y_arr = prior_y_arr(:)';
%     %         points(points_id, 15) = prior_y_arr(v + (u-1)*para.cam.height)';
%     %         prior_h_arr = prior_h_arr(:)';
%     %         points(points_id, 16) = prior_h_arr(v + (u-1)*para.cam.height)';
%     prior_int_arr = prior_int_arr(:)';
%     prior_int_pt = prior_int_arr(v + (u-1)*para.cam.height)';
%     prior_inteq_arr = prior_inteq_arr(:)';
%     prior_inteq_pt = prior_inteq_arr(v + (u-1)*para.cam.height)';
%     prior_r_arr = prior_r_arr(:)';
%     prior_r_pt = prior_r_arr(v + (u-1)*para.cam.height)';
%     prior_g_arr = prior_g_arr(:)';
%     prior_g_pt = prior_g_arr(v + (u-1)*para.cam.height)';
%     prior_b_arr = prior_b_arr(:)';
%     prior_b_pt = prior_b_arr(v + (u-1)*para.cam.height)';
% %     prior_y_arr = prior_y_arr(:)';
% %     prior_y_pt = prior_y_arr(v + (u-1)*para.cam.height)';
% %     prior_h_arr = prior_h_arr(:)';
% %     prior_h_pt = prior_h_arr(v + (u-1)*para.cam.height)';
%     toc
%     
%     tic
%     disp('4. save projection info');
%     for pt_idx=1:length(points_id)
% %         color_buf{points_id(pt_idx)} = [color_buf{points_id(pt_idx)}; ...
% %             img_idx, u(pt_idx), v(pt_idx), ...
% %             prior_int_pt(pt_idx), prior_inteq_pt(pt_idx), ...
% %             prior_y_pt(pt_idx), prior_h_pt(pt_idx)];
% color_buf{points_id(pt_idx)} = [color_buf{points_id(pt_idx)}; ...
%     img_idx, u(pt_idx), v(pt_idx), ...
%     prior_int_pt(pt_idx), prior_inteq_pt(pt_idx), ...
%     prior_r_pt(pt_idx), prior_g_pt(pt_idx), prior_b_pt(pt_idx)];
%     end
%     toc
% end
% 
% save('colorProjectionData.mat', 'color_buf');

% load('colorProjectionData.mat');

%% removing 3d points that are with changing color pattern
% for i=1:length(color_buf)
%     for pidx = 1:size(color_buf{i}, 1)
%         start_idx = pidx;
%         while start_idx <= size(color_buf{i}, 1)
%             
%             start_idx = start_idx + 1;
%         end
%     end
% end

%% visualization
setenv('OMP_NUM_THREADS', '4');

vis_idx = 10
image = img_list{vis_idx};
[tmp_x0, t, R, inv_R, ~] = addError(para.err_list{1}, para.x0, 1+step_num, step_num);
[u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
tic
[u_cpp, v_cpp, points_id_cpp] = occludedPointsRemoval_upd(u, v, points_id, image, points);
toc

% figure;
% imagesc(image.image);
% hold on;
% scatter(u, v, ones(1,length(u)), points(points_id, 6), 'filled');

% % test matlab approach
% tic
% [u_mat, v_mat, points_id_mat] = occludedPointsRemoval(u, v, points_id, image, points);
% toc
% figure;
% imagesc(image.image);
% hold on;
% scatter(u_mat, v_mat, ones(1,length(u_mat)), points(points_id_mat, 6), 'filled');
% 
% % test cpp approach
% tic
% [u_cpp, v_cpp, points_id_cpp] = occludedPointsRemoval_upd(u, v, points_id, image, points);
% toc
figure;
imagesc(image.image);
hold on;
scatter(u_cpp, v_cpp, ones(1,length(u_cpp)), points(points_id_cpp, 6), 'filled');

[uin,vin] = ginput(1);
u_diff = abs(uin-u);
v_diff = abs(vin-v);
[~, spt_idx] = min(u_diff + v_diff);

scatter(u(spt_idx), v(spt_idx), 3, 'rx');
figure;
plot(color_buf{points_id(spt_idx)}(:, 1), color_buf{points_id(spt_idx)}(:, 4), '-o');
hold on;
plot(color_buf{points_id(spt_idx)}(:, 1), color_buf{points_id(spt_idx)}(:, 5), '-+');
plot(color_buf{points_id(spt_idx)}(:, 1), color_buf{points_id(spt_idx)}(:, 6), '-*');
plot(color_buf{points_id(spt_idx)}(:, 1), color_buf{points_id(spt_idx)}(:, 7), '-x');
plot(color_buf{points_id(spt_idx)}(:, 1), color_buf{points_id(spt_idx)}(:, 8), '-s');
%     'int-int';
%     'int_{hist}-int_{hist}';
%     'int_{shadow}-int_{shadow}';
%     'hue-hue';
legend({'int';'int_{hist}';'red';'green';'blue';});
ylim([0 1])
% xlim([1 frame_limit])

available_img_buf = color_buf{points_id(spt_idx)}(:, 1);
if ~isempty(available_img_buf)
    for i = 1:length(available_img_buf)
        available_idx = available_img_buf(i);
        figure;imagesc(adapthisteq(rgb2gray(img_list{available_idx}.image)));
        hold on;
        scatter(color_buf{points_id(spt_idx)}(i, 2), color_buf{points_id(spt_idx)}(i, 3), 3, 'rx');
        title(['frame' num2str(color_buf{points_id(spt_idx)}(i, 1))]);
    end
end
