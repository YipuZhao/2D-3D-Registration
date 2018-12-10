function [u, v] = Cam2Img(points_camera, para)
if strcmp(para.camera_type, 'GT') || strcmp(para.camera_type, 'HWY')
    a = points_camera(1,:) ./ points_camera(2,:);
    b = -points_camera(3,:) ./ points_camera(2,:);
    r = sqrt(a.*a + b.*b);
    
    kc = para.cam.kc;
    fc = para.cam.fc;
    cc = para.cam.cc;
    
    r4 = r.^4;
    dpt0 = a .* (1 + kc(1)*(r.^2) + kc(2)*r4) + 2*kc(3)*a.*b + kc(4)*(r.^2 + 2*a.^2);
    dpt1 = b .* (1 + kc(1)*(r.^2) + kc(2)*r4) + kc(3)*(r.^2 + 2*b.^2) + 2*kc(4)*a.*b;
    
    u = fc(1)*dpt0+cc(1);
    v = fc(2)*dpt1+cc(2);
elseif strcmp(para.camera_type, 'Pandey')
    points_image = para.cam.K*points_camera;
    v = points_image(1,:)./points_image(3,:);
    u = points_image(2,:)./points_image(3,:);
elseif strcmp(para.camera_type, 'kitti')
    p2_out = (para.cam.to_img * points_camera)';
    p_out = p2_out(:,1:2)./(p2_out(:,3)*ones(1,2));
    u = p_out(:,1)';
    v = p_out(:,2)';
end
u = round(u);
v = round(v);