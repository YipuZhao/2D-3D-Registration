function [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points)

% remove occluded point
dist_thres = 1.0;
k = 50;
% r = 10;
fore_ratio = 0.1;
direc_ratio = 0.3;

camera_loc = image.xyz;
valid_idx = ones(size(u, 2), 1);
IDX = knnsearch([u', v'], [u', v'], 'K', k);
% IDX = rangesearch([u', v'], [u', v'], r);
for i=1:size(IDX,1)
%     i
    nn_idx = points_id(IDX(i, :));
    dist_arr = sqrt(sum((points(nn_idx(:), 2:4) - repmat(camera_loc, length(nn_idx), 1)).^2, 2));
    dist_tar = norm(points(points_id(i), 2:4) - camera_loc);
    % compare the distance between dist_arr & dist_tar
    fore_idx = find(dist_arr < dist_tar - dist_thres);
    if length(fore_idx) < k * fore_ratio
        continue ;
    end
    % check the distribution of foreground points
    fore_u = u(IDX(i, fore_idx));
    fore_v = v(IDX(i, fore_idx));
    sum_distri = sum(normc([fore_u - u(i); fore_v - v(i)]), 2);
    if norm(sum_distri) < length(fore_idx) * direc_ratio
        % remove current point
        valid_idx(i) = 0;
    end
end

% figure;
% subplot(1,2,1)
% imagesc(image.image)
% hold on
% scatter(u, v, 5, points(points_id, 5));

u = u(valid_idx == 1);
v = v(valid_idx == 1);
points_id = points_id(valid_idx == 1);


% subplot(1,2,2)
% imagesc(image.image)
% hold on
% scatter(u, v, 5, points(points_id, 5));
