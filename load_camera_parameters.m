function [cam] = load_camera_parameters(para)

if strcmp(para.camera_type, 'GT') || strcmp(para.camera_type, 'HWY')
    width           = 2448;
    height          = 2048;
    
    focal_length    = 8.51690061;
    camera_center_X = -0.0020583;
    camera_center_Y = 0.00465293;
    scale_x         = 0.00345;
    scale_y         = 0.00345;
    coef            = [0.00125859,0,0,-0.00010658,0,0];
    
    cam.kc = coef([1,2,4,5]);
    cam.fc = [focal_length/scale_x, focal_length/scale_y];
    cam.cc = [width/2+camera_center_X/scale_x, height/2+camera_center_Y/scale_y];
elseif strcmp(para.camera_type, 'Pandey')
    width           = 616;
    height          = 1616;
    focal_length    = 408.397136;
    camera_center_X = 806.586960;
    camera_center_Y = 315.535008;
    scale_x         = 1;
    scale_y         = 0.5;
    cam.K = [focal_length*scale_x, 0, camera_center_X
        0, focal_length*scale_y, camera_center_Y
        0, 0, 1];
elseif strcmp(para.camera_type, 'kitti')
    if ~strcmp(para.data_name, 'kitti0026')
        R_rect = [9.998817e-01 1.511453e-02 -2.841595e-03;
            -1.511724e-02 9.998853e-01 -9.338510e-04;
            2.827154e-03 9.766976e-04 9.999955e-01];
        P_rect = [7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01;
            0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01;
            0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03];
        R_cam_to_rect = eye(4);
        R_cam_to_rect(1:3,1:3) = R_rect;
        cam.to_img = P_rect*R_cam_to_rect;
        width = 1242;
        height = 375;
    else
        R_rect = [9.998896e-01 1.484154e-02 7.649204e-04;
                 -1.484114e-02 9.998897e-01 -5.289052e-04;
                 -7.726858e-04 5.174945e-04 9.999996e-01];
        P_rect = [7.183351e+02 0.000000e+00 6.003891e+02 4.450382e+01;
                  0.000000e+00 7.183351e+02 1.815122e+02 -5.951107e-01;
                  0.000000e+00 0.000000e+00 1.000000e+00 2.616315e-03];
        R_cam_to_rect = eye(4);
        R_cam_to_rect(1:3,1:3) = R_rect;
        cam.to_img = P_rect*R_cam_to_rect;
        width = 1238;
        height = 374;
    end
end
% cam.mask            = flipud(rgb2gray(imread(para.mask_name)));
cam.mask            = rgb2gray(imread(para.mask_name));
cam.width = width;
cam.height = height;