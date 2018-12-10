function [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R)
% points_world = [UTC, x, y, z, ...]
% t, inv_R: lidar to cam

UTC = image.UTC;
points_num = size(points,1);
points_UTC = points(:,1);
points_id = 1:points_num;

% get sub point cloud
points_idx = points_UTC >= UTC - para.time_expand_pre ...
    & points_UTC < UTC + para.time_expand_post;
points_num = sum(points_idx);
points_world = points(points_idx, 2:4)'; %[x, y, z]
points_id = points_id(points_idx);

% world to cam
if strcmp(para.camera_type, 'GT') || strcmp(para.camera_type, 'HWY')
    car_R = makeRotationMatrix(image.prh);
    car_inv_R = inv(car_R);
    car_t = image.xyz';
    points_car = car_inv_R * (points_world - repmat(car_t, [1, points_num]));
    points_camera = inv_R * (points_car - repmat(t, [1, points_num]));
    
    points_img_idx = points_camera(2,:)>1.5 & points_camera(2,:) < para.depth_thrd; % in front of cam
    points_camera = points_camera(:,points_img_idx);
    points_id = points_id(points_img_idx);
elseif strcmp(para.camera_type, 'Pandey')
    points_camera = R* points_world + repmat(t,[1, points_num]);
    points_img_idx = points_camera(3,:)>0; % in front of cam
    points_camera = points_camera(:,points_img_idx);
    points_id = points_id(points_img_idx);
elseif strcmp(para.camera_type, 'kitti')
    %
    car_R = makeRotationMatrix(image.prh);
    car_inv_R = inv(car_R);
    car_t = image.xyz';
    points_car = car_inv_R * (points_world - repmat(car_t, [1, points_num]));
    %
    Tr = [R t;0 0 0 1];
    %     points_world(4, :) = 1;
    %     points_camera = Tr * points_world;
    points_car(4, :) = 1;
    points_camera = Tr * points_car;
    %
    points_img_idx = points_camera(3,:)>=5;% & points_world(3,:)<-1.5 & points_world(3,:)>-2.3; % in front of cam
    points_camera = points_camera(:,points_img_idx);
    points_id = points_id(points_img_idx);
end

% cam to img
[u, v] = Cam2Img(points_camera, para);
points_img_idx = u>para.ROI_width(1) & u<= para.ROI_width(2) & v>para.ROI_height(1) & v<para.ROI_height(2);
u = u(points_img_idx);
v = v(points_img_idx);
points_id = points_id(points_img_idx);
points_img_idx = matVisitByUV(para.cam.mask,u,v)~=0;
u = u(points_img_idx);
v = v(points_img_idx);
points_id = points_id(points_img_idx);
