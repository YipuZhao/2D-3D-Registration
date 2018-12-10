function [v] = poseAssess(mi_cost, point_num_valid, thrd)
point_num = reshape([mi_cost.num],size(mi_cost,1),[]);
%valid_check
valid = ones(1,size(point_num,2));
for i = 1:size(point_num,1)
    valid = valid & point_num(i,:)>thrd*0.25;
end
valid_check = sum(~valid(point_num_valid));
if valid_check>0
    disp('got grid non-valid');
end
mi_cost(:,point_num_valid==0) = [];
mi = reshape([mi_cost.mi],size(mi_cost,1),[]);
H = reshape([mi_cost.H],size(mi_cost,1),[]);
v = get_combine_mi(H);
% v = sum(mi,2)/size(mi,2);