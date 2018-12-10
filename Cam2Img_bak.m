function [u, v] = Cam2Img(points_camera, para)

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

u = round(u);
v = round(v);