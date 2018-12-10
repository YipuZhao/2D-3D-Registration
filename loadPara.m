function [para] = loadPara(data_name, sm_name, err_type, sm_type)
%%
para.sm_list = sm_name;
para.err_list = err_type;
para.data_name = data_name;
% map = 'gray', 'cur'
% para.map = map;
para.MI_all = 0;

para.SM_type = sm_type; %'MI' %'NMI'

%%
switch data_name
    case 'site1'
        para.dir_image_name = '../../Data/20150706_CCW_Run1(0)/Site1/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.4385447877981982,	-0.13670360974551396,	0.34212599914835345,...
            -0.1024694779910818,	-0.01372107609625726,	-0.3431133549680061,...
            ];
    case 'site2'
        para.dir_image_name = '../../Data/20150706_CCW_Run1(0)/Site2/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47266705684149446,	-0.09386531498203768,	0.38869972087681987,...
            -0.10718650224342938,	-0.013880943773384209,	-0.3447841875216869,...
            ];
    case 'site3'
        para.dir_image_name = '../../Data/20150706_CCW_Run1(0)/Site3/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47245612336072884,	-0.09253817736061551,	0.34523804588277557,...
            -0.10381674038514718,	-0.01449061189809949,	-0.34599858664229716,...
            ];
    case 'site4'
        para.dir_image_name = '../../Data/20150706_CCW_Run1(0)/Site4/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47760411876328945,	-0.09395197513063053,	0.365188083707862,...
            -0.10428232822960384,	-0.018757738825691355,	-0.34692125864958445,...
            ];
        %
    case 'site2_run2'
        para.dir_image_name = '../../Data/20150706_CCW_Run2(0)/Site2/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47266705684149446,	-0.09386531498203768,	0.38869972087681987,...
            -0.10718650224342938,	-0.013880943773384209,	-0.3447841875216869,...
            ];
    case 'site3_run2'
        para.dir_image_name = '../../Data/20150706_CCW_Run2(0)/Site3/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47266705684149446,	-0.09386531498203768,	0.38869972087681987,...
            -0.10718650224342938,	-0.013880943773384209,	-0.3447841875216869,...
            ];
        %
    case 'site2_run3'
        para.dir_image_name = '../../Data/20150710_CW_Run1(0)/Site2/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47266705684149446,	-0.09386531498203768,	0.38869972087681987,...
            -0.10718650224342938,	-0.013880943773384209,	-0.3447841875216869,...
            ];
    case 'site3_run3'
        para.dir_image_name = '../../Data/20150710_CW_Run1(0)/Site3/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.47266705684149446,	-0.09386531498203768,	0.38869972087681987,...
            -0.10718650224342938,	-0.013880943773384209,	-0.3447841875216869,...
            ];
        %
    case 'kitti0002'
        para.dir_image_name = '../../Data/kitti/2011_09_26_drive_0002_sync/image_02/data/';
        para.dir_velo_name = '../../Data/kitti/2011_09_26_drive_0002_sync/velodyne_points/data/';
        para.camera_type = 'kitti';
%         para.image_end = 76;
        para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    case 'kitti0093'
        para.dir_image_name = '../../Data/kitti/2011_09_26_drive_0093_sync/image_02/data/';
        para.dir_velo_name = '../../Data/kitti/2011_09_26_drive_0093_sync/velodyne_points/data/';
        para.camera_type = 'kitti';
        para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    case 'kitti0104'
        para.dir_image_name = '../../Data/kitti/2011_09_26_drive_0104_sync/image_02/data/';
        para.dir_velo_name = '../../Data/kitti/2011_09_26_drive_0104_sync/velodyne_points/data/';
        para.camera_type = 'kitti';
        para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    case 'kitti0106'
        para.dir_image_name = '../../Data/kitti/2011_09_26_drive_0106_sync/image_02/data/';
        para.dir_velo_name = '../../Data/kitti/2011_09_26_drive_0106_sync/velodyne_points/data/';
        para.camera_type = 'kitti';
        para.x0 = [-4.069766e-03, -7.631618e-02, -2.717806e-01, 0.0075,   -1.5560,    1.5701];
    %
    case 'change_ref0_h'
        para.dir_image_name = '../../Data/20140530_Sign_Change/run3_reference_12ft(0)/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.523872211177982,-0.104662937750484,0.307160845021077,...
            -0.0987286844544612,-0.0122763281760458,-0.281873001610084...
            ];
    case 'change_tilt0'
        para.dir_image_name = '../../Data/20140530_Sign_Change/run4_xtilt15_12ft(0)/FR/';
        para.camera_type = 'GT';
        para.x0 = [...
            -0.523872211177982,-0.104662937750484,0.307160845021077,...
            -0.0987286844544612,-0.0122763281760458,-0.281873001610084...
            ];
        %
    case 'I475N_150529'
        para.dir_image_name = '../../Data/20150529_I475/NB/FR/';
        para.camera_type = 'HWY';
        para.x0 = [...
            -0.508636198390449,-0.159940247172629,0.316907822759424,...
            -0.109178352323250,-0.0125862938433134,0.0130734409406668...
            ];
    case 'I475S_150529'
        para.dir_image_name = '../../Data/20150529_I475/SB/FR/';
        para.camera_type = 'HWY';
        para.x0 = [...
            -0.508636198390449,-0.159940247172629,0.316907822759424,...
            -0.109178352323250,-0.0125862938433134,0.0130734409406668...
            ];
        %
    case 'I475N_150326'
        para.dir_image_name = '../../Data/20150326_I475/NB/FR/';
        para.camera_type = 'HWY';
        para.x0 = [...
            -0.490433524032043,-0.148155949104531,0.321778196395224,...
            -0.108326798409995,-0.0137125490099005,0.00947282510907372...
            ];
    case 'I475S_150326'
        para.dir_image_name = '../../Data/20150326_I475/SB/FR/';
        para.camera_type = 'HWY';
        para.x0 = [...
            -0.490433524032043,-0.148155949104531,0.321778196395224,...
            -0.108326798409995,-0.0137125490099005,0.00947282510907372...
            ];
    case 'calib_150326'
        para.dir_image_name = '../../Data/20150326_I475/Calib/FR/';
        para.camera_type = 'HWY';
        para.x0 = [...
            -0.504678232509733,-0.152570843048771,0.335783451410465,...
            -0.107358335277849,-0.0124074801004342,0.0100362467993671...
            ];
end

para.frame_use_num     = -1; %-1 = get all
para.image_skip_num    = 1;
% para.grid_row_num      = 3;%2;
% para.grid_col_num      = 4;%3;
% para.grid_row_num      = 5;
% para.grid_col_num      = 6;
para.grid_row_num      = 2;
para.grid_col_num      = 3;

para.depth_thrd = 100;
% para.cam = load_camera_parameters(para);
para.bin_fraction = 1;%2; %4; % bins size = 256/bin_fraction
% if strcmp(map, '2feature')
%     para.bin_fraction = 8;
% end
para.point_thres = 1000;
para.GRID_THRD = 400;
para.AREA_THRD = 22;
para.do_registration = 5; %???
para.pavement_cut = 0;
para.GPS_name = 'Cam_GPS.txt';
para.point_cloud_name = 'Scan_for_MI.txt';

if strcmp(para.camera_type, 'GT') %strcmp(para.camera_type, 'GT_V') || strcmp(para.camera_type, 'GT_H')
    para.dir_name = '../../Data/GT_reorg/';
    para.dir_sm = [para.dir_name 'similarity/'];
    para.dir_reg = [para.dir_name 'registration/'];
%     para.dir_sm = [para.dir_name 'similarity\'];
    %     para.dir_sm = [para.dir_name 'validate\'];
    para.mask_name = [para.dir_name 'GT_mask_FR.jpg'];
    para.image_name_format = 'image_%06d.jpg';
%     para.label_name_format = 'label_%06d.txt';
    %     para.point_cloud_name = 'Scan_for_MI.txt';
    %     para.point_seg_name = 'depth_seg.txt';
    para.cam = load_camera_parameters(para);
    
    para.time_expand_pre         = 2;%4;%3; % point cloud used for calculation
    para.time_expand_post        = 4;%4;%3;
    para.ROI_height = [1, para.cam.height];
    para.ROI_width = [1, para.cam.width];
    degree = 0;
    % ground truth
    %     para.x0 = [-0.439985, -0.099087, 0.369588, -0.104502, -0.014067, -0.351024];
    % biased initial parameters
    %     para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698];
    % 20150905 results
    %     para.x0 = [-0.468551172430471,-0.0896223075945281,0.362394204109037,-0.102802643168298,-0.0170526283372424,-0.347804035418936];
    % para.x0 = [-0.456577, -0.119576, 0.381374, -3.10136, -0.016340, -0.344929]; degree = 0;
    % para.x0 = [-0.484184, -0.111809, 0.421773, -0.104680, -0.014401, -0.345698]; degree = 0;
    %     para.x0 = [-0.471949, -0.133226, 0.404574, -0.106879, -0.015601, -0.345973]; %b
    % para.x0 = [-0.479093, -0.103110, 0.375047, -0.106241, -0.017385, -0.346278];
elseif strcmp(para.camera_type, 'HWY') %strcmp(para.camera_type, 'HWY_H') || strcmp(para.camera_type, 'HWY_V')
    para.dir_name = '../../Data/Hwy_reorg/';
    %   para.dir_sm = [para.dir_name 'noise_segment\'];
    para.dir_sm = [para.dir_name 'similarity/'];
    para.dir_reg = [para.dir_name 'registration/'];
    %     para.dir_sm = [para.dir_name 'validate\'];
    para.mask_name = [para.dir_name 'GT_mask_FR.jpg'];
    para.image_name_format = 'image_%06d.jpg';
%     para.label_name_format = 'label_%06d.txt';
    %     para.point_cloud_name = 'Scan_for_MI.txt';
    %     para.point_seg_name = 'depth_seg.txt';
    para.cam = load_camera_parameters(para);
    
    para.time_expand_pre         = 1;%1;%4;%3; % point cloud used for calculation
    para.time_expand_post        = 2;%1.5;%4;%3;
    para.ROI_height = [1, para.cam.height];
    para.ROI_width = [1, para.cam.width];
    degree = 0;
elseif strcmp(para.camera_type, 'kitti')
    para.dir_name = '../../Data/kitti/';
    para.dir_sm = [para.dir_name 'similarity/'];
    para.dir_reg = [para.dir_name 'registration/'];
    % para.dir_sm = [para.dir_name 'example\'];
    para.mask_name =[para.dir_name  'mask.jpg'];
    para.image_name_format = '%010d.png';
    %     para.point_cloud_format = '%s%010d_new.bin';
    para.label_name_format = 'label_%010d.txt';
    %
    %     if strcmp(data_name, 'kitti_road')
    %         para.image_name_format = 'umm_%06d.png';
    %         para.point_cloud_format = '%summ_%06d_new.bin';
    %     end
    para.cam = load_camera_parameters(para);
    
    para.time_expand_pre         = 2.1;%0.1; % point cloud used for calculation
    para.time_expand_post        = 2.1;%0.1;
    para.ROI_height = [101, para.cam.height];
    para.ROI_width = [1, para.cam.width];
    degree = 0;
    para.x0 = [-0.025270, -0.072682, -0.280618, 0.003881, -1.559390, 1.571956];
    %     para.x0 = [-0.025270, -0.072682, -0.280618, 0.003881, -1.559390, 1.571956];
    %     para.x0 = [-0.028985, -0.082089, -0.273588, 0.003288, -1.559157, 1.568977]; %best
    %     para.x0 = [-0.037143, -0.082385, -0.261382, 0.004269, -1.557891, 1.571840];%all
    %     para.x0 = [-0.020143, 0.090711, -0.357840, 0.000473, -1.521707, 1.577686];
    %     para.x0 = [-0.042057, -0.063591, -0.283596, 0.005148, -1.560508, 1.571825];
    %     para.x0 = [-0.477399, -0.167731, 0.329471, -0.104290, -0.018124, -0.346884];
end

para.dir_output = [para.dir_reg, para.data_name '_' 'output/'];
para.grid_height = floor((para.ROI_height(2)-para.ROI_height(1)) / para.grid_row_num);
para.grid_width = floor((para.ROI_width(2)-para.ROI_width(1)) / para.grid_col_num);
if degree
    para.x0(4:6) = para.x0(4:6)/180*pi;
end