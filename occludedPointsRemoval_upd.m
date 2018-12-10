function [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points)
%%
[u, v, points_id] = OcclPointRemoval_cpp(length(points_id), u, v, points_id, image.xyz, ...
    points(:, 2), points(:, 3), points(:, 4), 0.1, 0.6);
