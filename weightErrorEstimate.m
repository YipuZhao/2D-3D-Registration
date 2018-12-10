function [sm] = weightErrorEstimate(voxel_weight_arr)
%%
global reg_img_list
global reg_points
global para
global w
global point_voxel_arr
global weight_count

voxel_weight_arr = voxel_weight_arr / sum(voxel_weight_arr) * weight_count;

disp('compute weighted mi---------');
tic
% sm_allframe = mi_cost_compute(x0, reg_img_list, reg_points, para, w);
sm_allframe = weighted_mi_cost_compute(para.x0, reg_img_list, reg_points, point_voxel_arr, voxel_weight_arr, para, w);
toc

sm = -sum(sm_allframe);