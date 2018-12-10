function [] = get_probability(MI_hist)
mu_gray = MI_hist.gray_sum/MI_hist.count;
mu_refc = MI_hist.refc_sum/MI_hist.count;