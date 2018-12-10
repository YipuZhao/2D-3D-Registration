function [nmi_cost, mi_cost, point_num, entropy_joint] = get_NMI(MI_prob, nbins)
if (MI_prob.count < 2)
    nmi_cost = 0;
    mi_cost = 0;
    point_num = 0;
    entropy_joint = zeros(nbins);
else
    entropy_gray = log(MI_prob.prob_gray) .* MI_prob.prob_gray;
    entropy_refc = log(MI_prob.prob_refc) .* MI_prob.prob_refc;
    entropy_joint = log(MI_prob.prob_joint) .* MI_prob.prob_joint;
    
    entropy_gray(MI_prob.prob_gray==0) = 0;
    entropy_refc(MI_prob.prob_refc==0) = 0;
    entropy_joint(MI_prob.prob_joint==0) = 0;
    
    Hx = norm(entropy_gray,1);
    Hy = norm(entropy_refc,1);
    Hxy = norm(entropy_joint(:),1);
    nmi_cost = (Hx + Hy)/Hxy;
    mi_cost = Hx + Hy - Hxy;
    point_num = MI_prob.count;
end