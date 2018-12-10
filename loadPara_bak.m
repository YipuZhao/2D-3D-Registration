function [para] = loadPara(data_name, sm_name, err_type)
%%
para.sm_list = sm_name;
para.err_list = err_type;
para.data_name = data_name;
% map = 'gray', 'cur'
% para.map = map;
para.MI_all = 0;
if strcmp(data_name, 'site1')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20150706_CCW_Run1(0)\Site1\FC\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FC.jpg';
    para.image_name_format = 'image_%06d.jpg';
    % para.x0  = [-0.878, -0.156, 0.732, -19, 0, 1];
    para.x0 = [-0.725313, -0.049884, 0.853310, -0.346825, -0.002669, 0.041284];
    degree = 0;
elseif strcmp(data_name, 'site1_v')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20150706_CCW_Run1(0)\Site1\FR\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FR.jpg';
    para.image_name_format = 'image_%06d.jpg';
%     para.x0 = [-0.483537, -0.087225, 0.387998, -0.108476, -0.014568, -0.350494];
para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698];
    degree = 0;
elseif strcmp(data_name, 'site2_v')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20150706_CCW_Run1(0)\Site2\FR\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FR.jpg';
    para.image_name_format = 'image_%06d.jpg';
    %     para.x0 = [-0.483537, -0.087225, 0.387998, -0.108476, -0.014568, -0.350494];
%     para.x0 = [-0.508, -0.085, 0.392, -6, 0, -21.4];
% para.x0 = [-0.485382, -0.115236, 0.398933, -0.107525, -0.014124, -0.346663];
para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698];
    degree = 0;
elseif strcmp(data_name, 'site3_v')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20150706_CCW_Run1(0)\Site3\FR\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FR.jpg';
    para.image_name_format = 'image_%06d.jpg';
    %     para.x0 = [-0.483537, -0.087225, 0.387998, -0.108476, -0.014568, -0.350494];
    %     degree = 0;
%     para.x0 = [-0.446038, -0.099770, 0.379107, -0.106589, -0.015273, -0.351204];
para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698];
    %     para.x0 = [-0.532041546551560, -0.008570378020896, 0.337752308775172, ...
    % %         -0.106589, -0.015273, -0.351204
    %         -0.104719755119660, 0, -0.373500459926787
    %     ];
    degree = 0;
    %     para.x0 = [-0.508, -0.085, 0.304, -6, 0, -21.1];
    %     degree = 1;
elseif strcmp(data_name, 'site4_v')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20150706_CCW_Run1(0)\Site4\FR\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FR.jpg';
    para.image_name_format = 'image_%06d.jpg';
    %     para.x0 = [-0.483537, -0.087225, 0.387998, -0.108476, -0.014568, -0.350494];
    %     para.x0 = [-0.508, -0.085, 0.304, -6, 0, -21.1];
    %     degree = 1;
%     para.x0 = [-0.446038, -0.099770, 0.379107, -0.106589, -0.015273, -0.351204];
    para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698];
% para.x0 = [-0.467161898323445,-0.138974294211078,0.407628471671303,-0.107713263282377,-0.0155421805798572,-0.346644993841738];
    degree = 0;
elseif strcmp(data_name, 'run3_v')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\20140613_Clock_Run1(0)\Site3\FR\';
    para.camera_type = 'GT';
    para.mask_name = '.\data\GT_mask_FR.jpg';
    para.image_name_format = 'image_%06d.jpg';
    para.x0 = [-0.512880, -0.077987, 0.337610, -0.103204, -0.011544, -0.303344];
    degree = 0;
elseif strcmp(data_name, 'Pandey')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\literature code\mi-extrinsic-calib-data\Cam0\';
    para.camera_type = 'Pandey';
    para.mask_name = '..\literature code\mi-extrinsic-calib-data\Mask\Cam0.png';
    para.image_name_format = 'image%04d.ppm';
    para.x0 = [0.4, 0, 0.3, 90, 0, -90];
    degree = 1;
elseif strcmp(data_name, 'kitti0002')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0002_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0002_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.image_end = 76;
    para.point_cloud_format = '%s%010d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0026')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_29_drive_0026_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_29_drive_0026_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask_29.jpg';
    para.image_name_format = '%010d.png';
    para.image_end = 76;
    para.point_cloud_format = '%s%010d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0060')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0060_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0060_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.point_cloud_format = '%s%010d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0084')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0084_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0084_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.point_cloud_format = '%s%010d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0093')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0093_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0093_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.point_cloud_format = '%s%010d.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0104')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0104_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0104_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.point_cloud_format = '%s%010d.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti0106')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\2011_09_26_drive_0106_sync\image_02\data\';
    para.dir_velo_name = '..\..\data\kitti\2011_09_26_drive_0106_sync\velodyne_points\data\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = '%010d.png';
    para.point_cloud_format = '%s%010d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
elseif strcmp(data_name, 'kitti_road')
    para.dir_name = '.\data\';
    para.dir_image_name = '..\..\data\kitti\road\image_02\';
    para.dir_velo_name = '..\..\data\kitti\road\velodyne_points\';
    para.camera_type = 'kitti';
    para.mask_name = '..\..\data\kitti\mask.jpg';
    para.image_name_format = 'umm_%06d.png';
    para.point_cloud_format = '%summ_%06d_new.bin';
    para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    degree = 0;
end
para.dir_output = ['..\..\data\', data_name '_' 'output\'];

para.frame_use_num     = -1; %-1 = get all
para.image_skip_num    = 2;
% para.grid_row_num      = 3;%2;
% para.grid_col_num      = 4;%3;
% para.grid_row_num      = 5;
% para.grid_col_num      = 6;
para.grid_row_num      = 2;
para.grid_col_num      = 3;

para.depth_thrd = 100;
para.cam = load_camera_parameters(para);
para.bin_fraction = 1;%2; %4; % bins size = 256/bin_fraction
% if strcmp(map, '2feature')
%     para.bin_fraction = 8;
% end
para.GRID_THRD = 400;
para.AREA_THRD = 22;
para.do_registration = 5;
para.pavement_cut = 0;
para.GPS_name = 'Cam_GPS.txt';
para.point_cloud_name = 'Scan_for_MI.txt';

if strcmp(para.camera_type, 'GT')
    para.time_expand_pre         = 3; % point cloud used for calculation
    para.time_expand_post        = 3;%3;
    para.ROI_height = [1, para.cam.height];
    para.ROI_width = [1, para.cam.width];
elseif strcmp(para.camera_type, 'Pandey')
    para.time_expand_pre         = 0.1; % point cloud used for calculation
    para.time_expand_post        = 0.1;
    para.ROI_height = [750, 1050];
    para.ROI_width = [120, 500];
elseif strcmp(para.camera_type, 'kitti')
    para.time_expand_pre         = 0.1; % point cloud used for calculation
    para.time_expand_post        = 0.1;
    para.ROI_height = [101, para.cam.height];
    para.ROI_width = [1, para.cam.width];
    %     para.x0 = [-0.025270, -0.072682, -0.280618, 0.003881, -1.559390, 1.571956];
    para.x0 = [-0.028985, -0.082089, -0.273588, 0.003288, -1.559157, 1.568977];
    %     para.x0 = [-0.037143, -0.082385, -0.261382, 0.004269, -1.557891, 1.571840];%all
    % para.x0 = [-0.020143, 0.090711, -0.357840, 0.000473, -1.521707, 1.577686];
    % para.x0 = [-0.042057, -0.063591, -0.283596, 0.005148, -1.560508, 1.571825];
end

para.grid_height = floor((para.ROI_height(2)-para.ROI_height(1)) / para.grid_row_num);
para.grid_width = floor((para.ROI_width(2)-para.ROI_width(1)) / para.grid_col_num);
if degree
    para.x0(4:6) = para.x0(4:6)/180*pi;
end