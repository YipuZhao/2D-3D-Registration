function w = weightOptimization(cluster)
% global err_num
% global sm_image_buf
global cur_cluster
% global step_num
global sm_num
%%
% w = zeros(1, sm_num);
% for j=1:length(cluster.patch_idx)
%     pidx = cluster.patch_idx(j);
%     for sno=1:sm_num
%         w(sno) = w(sno) + sum(sm_grid_buf{pidx}.score(:, sno));
%     end
% end
% w = (w - min(w)) / (max(w) - min(w));
% w = w / sum(w);

%%
cur_cluster = cluster;
% err_num = 6;
% step_num = 20;
MAX_ITER = 500;
w = ones(1, sm_num);
w = w / sum(w);
delta_w = 0.001;
step_length = 0.1;

%%
w = fminsearchbnd(@scoreCluster, w, zeros(1, sm_num), ones(1, sm_num), ...
    optimset('PlotFcns', @optimplotfval, 'MaxIter', 5000, 'MaxFunEvals', 10000));

% for index = 1 : MAX_ITER
%     score_ori = 0;
%     score_inc = zeros(1, length(w));
%     score_dec = zeros(1, length(w));
%     for j=1:length(cluster.patch_idx)
%         pidx = cluster.patch_idx(j);
%         trend_cur(:, :) = sum(sm_patch_buf{pidx}.trend(:, :, :), 1) / err_num;
%         %
%         %         trend_ori = w * trend_cur(:, :)';
%         %         score_ori = score_ori + scoreEstimation(trend_ori, step_num);
%         %
%         for k=1:length(w)
%             w_tmp = w;
%             w_tmp(k) = w_tmp(k) + delta_w;
%             trend_inc = w_tmp * trend_cur(:, :)';
%             score_inc(k) = score_inc(k) + scoreEstimation(trend_inc, step_num);
%         end
%         %
%         for k=1:length(w)
%             w_tmp = w;
%             w_tmp(k) = w_tmp(k) - delta_w;
%             trend_dec = w_tmp * trend_cur(:, :)';
%             score_dec(k) = score_dec(k) + scoreEstimation(trend_dec, step_num);
%         end
%     end
%     score_inc = score_inc / length(cluster.patch_idx);
%     score_dec = score_dec / length(cluster.patch_idx);
%     %
%     %     w = w + (score_inc > score_dec) * delta_w;
%     %     w = w - (score_inc < score_dec) * delta_w;
%     w = w + (score_inc - score_dec) / (2*delta_w) * step_length;
%     w = max(w, 0);
%     w = w / sum(w);
% end
% index
% w = (w - min(w)) / (max(w) - min(w));
w = w / sum(w);