close all;
clear all;
clc
data_list = {...
    'site1';
    'site1_v';
    'site2_v';
    'site3_v';
    'site4_v';
    %     'kitti0002';
    %     'kitti0026';
    %     'kitti0060';
    %     'kitti0084';
    %     'kitti0093';
    %     'kitti0104';
    %     'kitti0106';
    %     'kitti_road';
    };
% sm_list = {...
%     'Inten-RefEQ'; 'H-RefEQ'; 'V-RefEQ';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     'H-Nx'; 'H-Ny'; 'H-Nz';
%     'GradD-Curv'; 'GradM-Curv';
%     };
% sm_list = {
%      'Inten-Ref'; 'IntenEQ-Ref'; 'Y-Ref'; 'H-Ref';
%      'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%      'IntenEQ-Nx'; 'IntenEQ-Ny'; 'IntenEQ-Nz';
%      'GradMHue-CurvComb'; 'GradDHue-CurvComb'; 'EdgeInt-CurvComb';
%     };
sm_list = {
    'GradMHue-CurvComb'; 'GradMHue-GradMRef'; 'GradMHue-GradMComb';
    };

% err_list = {...
%     'heading';
%     'rolling';
%     'pitching';
%     'shift_x';
%     'shift_y';
%     'shift_z';
%     };

err_list = {...
    'heading';
    };

sm_type = 'NMI';

select_frame_list = {0, 8, 8, 8, 8};
% select_frame_list = {0, 39, 23, 57, 4};
% select_frame_list = {13, 47, 9};

for di=2:5
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    testSMTrend(img_list{select_frame_list{di}}, points, para);
    
    % para.x0 = gradient_descent_search_grid (para.x0, img_list, points, para);
    % mi_cost_2d(para.x0, img_list, points, para,'test_rst');
end