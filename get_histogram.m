function [MI_hist, MI_prob] = get_histogram(points_gray, points_refc, nbins)
if (length(points_gray) == 0)
    MI_hist.hist_gray = [];
    MI_hist.hist_refc = [];
    MI_hist.hist_joint = [];
    MI_hist.count = 0;
    MI_hist.gray_sum = 0;
    MI_hist.refc_sum = 0;
    
    MI_prob.prob_gray = [];
    MI_prob.prob_refc = [];
    MI_prob.prob_joint = [];
    MI_prob.count = 0;
else
%     disp('1------');
%     tic
    edge_gray = linspace(min(points_gray),max(points_gray),nbins+1);
    edge_refc = linspace(min(points_refc),max(points_refc),nbins+1);
    [hist_gray, edge_gray, bin_idx_gray] = histcounts(points_gray,edge_gray);
    [hist_refc, edge_refc, bin_idx_refc] = histcounts(points_refc,edge_refc);
    gray_refc = (bin_idx_gray-1)*nbins + bin_idx_refc;
    edge_gray_refc = linspace(0.5,nbins*nbins+0.5, nbins*nbins+1);
    hist_joint = histcounts(gray_refc, edge_gray_refc);
    hist_joint = reshape(hist_joint,nbins,nbins);
%     toc
    
%     disp('2------');
%     tic
    count = length(points_gray);
    gray_sum = sum(bin_idx_gray);
    refc_sum = sum(bin_idx_refc);
    
    MI_hist.hist_gray = hist_gray;
    MI_hist.hist_refc = hist_refc;
    MI_hist.hist_joint = hist_joint;
    MI_hist.count = count;
    MI_hist.gray_sum = gray_sum;
    MI_hist.refc_sum = refc_sum;
    
    C = cov(bin_idx_gray, bin_idx_refc);
    gray_var = C(1,1);
    refc_var = C(2,2);
    joint_var = C(1,2);
    corr_coeff = abs((joint_var)/(gray_var*refc_var));
    
    sigma_gray = 1.06*sqrt(gray_var)/(count^0.2);
    sigma_refc = 1.06*sqrt(refc_var)/(count^0.2);
    
%     prob_gray = gpuArray(hist_gray/count);
%     prob_refc = gpuArray(hist_refc/count);
%     prob_joint = gpuArray(hist_joint/count);
    prob_gray = (hist_gray/count);
    prob_refc = (hist_refc/count);
    prob_joint = (hist_joint/count);
    
%     toc
    
%     disp('3------');
%     tic
    if (sigma_gray>0 && sigma_refc>0)
%         disp('1D------');
%         tic
%         prob_gray = conv (prob_gray, gaussFilter, 'same');
        prob_gray = imgaussfilt(prob_gray, [1, sigma_gray]);
        prob_refc = imgaussfilt(prob_refc, [1, sigma_refc]);
%         toc
%         disp('2D------');
%         tic
        prob_joint = imgaussfilt(prob_joint, [sigma_refc, sigma_gray]);
%         tocR
    end
    
    MI_prob.prob_gray = prob_gray;
    MI_prob.prob_refc = prob_refc;
    MI_prob.prob_joint = prob_joint;
    MI_prob.count = count;
%     toc
end
