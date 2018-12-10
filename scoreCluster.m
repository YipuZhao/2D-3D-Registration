function sc = scoreCluster(w)
%%
global err_num
global sm_image_buf
global cur_cluster
% global step_num

w = w / sum(w);

sc = 0;
for j=1:length(cur_cluster.patch_idx)
    pidx = cur_cluster.patch_idx(j);
    trend_cur(:, :) = sum(sm_image_buf{pidx}.trend(:, :, :), 1) / err_num;
    %
    sc = sc - scoreEstimation(w * trend_cur(:, :)');
end