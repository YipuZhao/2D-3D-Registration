function [cam] = load_camera_parameters()
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
        
cam.width = width;
cam.height = height; 