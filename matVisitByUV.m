function [value] = matVisitByUV(mat, u, v)
height = size(mat,1);
vector = mat(:)';
idx = v + (u-1)*height;
value = vector(idx)';