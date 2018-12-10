calib_camera.shift_sensor = [0, 0.102, 0.23];
calib_camera.shift_support = [-0.508, -0.187, 0.162];
calib_camera.rotate_sensor = [deg2rad(-6), deg2rad(0), deg2rad(-21.4)];

calib_camera.rotate_matrix = makeRotationMatrix(calib_camera.rotate_sensor);
calib_camera.inv_rotate_matrix = inv(calib_camera.rotate_matrix);

calib_camera.shift_vector = calib_camera.shift_sensor + calib_camera.shift_support;
%(calib_camera.rotate_matrix * calib_camera.shift_sensor' + calib_camera.shift_support')';

vpt = vpt';
cpt = calib_camera.inv_rotate_matrix * (vpt - repmat(calib_camera.shift_vector', 1, size(vpt,2)));
cpt = cpt';

% wx wy wz 741504.743852 3740065.012669 253.789823
% frame 847
% img x y 
% vx vy vz -84.392021 33.773333 252.76
% vh vr vp 181.1 -0.3 1.3