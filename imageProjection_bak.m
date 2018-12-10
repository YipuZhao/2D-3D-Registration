function [u, v, points_id] = imageProjection(points_world, image, para, t, inv_R)
% points_world = [UTC, x, y, z, ...]
% t, inv_R: lidar to cam

UTC = image.UTC;
car_R = makeRotationMatrix(image.prh);
car_inv_R = inv(car_R);
car_t = image.xyz';

points_num = size(points_world,1);
points_UTC = points_world(:,1);
points_id = 1:points_num;

% get sub point cloud
points_idx = points_UTC >= UTC - para.time_expand_pre ...
    & points_UTC < UTC + para.time_expand_post;
points_num = sum(points_idx);
points_world = points_world(points_idx, 2:4)'; %[x, y, z]
points_id = points_id(points_idx);

% world to cam
points_car = car_inv_R * (points_world - repmat(car_t, [1, points_num]));
points_camera = inv_R * (points_car - repmat(t, [1, points_num]));
points_img_idx = points_camera(2,:)>0; % in front of cam
points_camera = points_camera(:,points_img_idx);
points_id = points_id(points_img_idx);

% cam to img
[u, v] = Cam2Img(points_camera, para);
points_img_idx = u>1 & u<para.cam.width & v>1 & v<para.cam.height-150;
u = u(points_img_idx);
v = v(points_img_idx);
points_id = points_id(points_img_idx);


