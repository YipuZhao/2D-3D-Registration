function score = scoreEstimation(trend)
%%
global step_num
alpha = 0.3;%0.001;0.07;%200
beta = 20;
base_w = 1.0;
wing_size = int32(step_num * 0.8);
x = double([-step_num + wing_size : step_num - wing_size]);
w = -base_w / double(step_num-wing_size)^2 * x.^2 + base_w;
%
% x = [-step_num : -step_num + wing_size - 1, x, step_num - wing_size + 1 : step_num];
w = [zeros(1, wing_size), w, zeros(1, wing_size)];
% w = min(x / step_num + 1, -x / step_num + 1);

max_nmi = max(trend);
peak_idx = find(trend == max_nmi);
[~, x_idx] = min(peak_idx - (step_num+1));
max_x = peak_idx(x_idx);
%                     [max_y, max_x] = max(sm_grid_buf{k}.trend(eno, :, sno));
score_bias = w(max_x);
%
sm_der = gradient(trend);
if max_x > 1
    der1 = sign(sm_der(1:max_x-1));
%     score_d1 = sum(der1(der1 < 0));
%     m1 = mean(der1);
    d1 = std(der1);
    score_d1 = - d1;% / abs(m1);
else
    score_d1 = 0;
end
if max_x < 1 + 2*step_num
    der2 = sign(sm_der(max_x:end));
%     score_d2 = -sum(der2(der2 > 0));
%     m2 = mean(der2);
    d2 = std(der2);
    score_d2 = - d2;% / abs(m2);
else
    score_d2 = 0;
end
score_smooth = score_d1 + score_d2;
% score_smooth = (m1 + m2) + (d1 + d2);
%
score_peak = max_nmi - mean(trend);
%
% score = max(0, score_bias + alpha * score_smooth + beta * score_peak);
score = score_bias + alpha * score_smooth + beta * score_peak;