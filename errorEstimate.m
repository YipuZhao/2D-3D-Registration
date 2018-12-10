function [sm] = errorEstimate(x0)
%%
global reg_img_list
global reg_points
global para
global w

sm_avgframe = mi_cost_compute(x0, reg_img_list, reg_points, para, w);
sm = -sm_avgframe;