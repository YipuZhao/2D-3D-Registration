close all;
clear all;
% clc
data_list = {...
    'site1';
    'site2';
    'site3';
    'site4';
%     'site1_v';
%     'site2_v';
%     'site3_v';
%     'site4_v';
    %     'run3_v';
    %     'Pandey';
    'kitti0002';
%     'kitti0093';
    'kitti0104';
    'kitti0106';
    %
    'I475N_150326';
    'I475S_150326';
    };
% sm_list = {...
%     'Inten-RefEQ'; 'H-RefEQ'; 'V-RefEQ';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     'H-Nx'; 'H-Ny'; 'H-Nz';
%     'GradD-Curv'; 'GradM-Curv';
%     };
% sm_list = {...
%     'Inten-RefEQ'; 'Y-RefEQ'; 'Cb-RefEQ'; 'Cr-RefEQ';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     'GradD-Curv'; 'GradM-Curv'; 'GradDEQ-Curv'; 'GradMEQ-Curv'; 'GradDFil-Curv'; 'GradMFil-Curv';
%     };
% sm_list = {...
%     'Inten-Ref'; 'IntenEQ-Ref'; 'Y-Ref'; 'H-Ref';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     %     'IntenEQ-Nx'; 'IntenEQ-Ny'; 'IntenEQ-Nz';
%     %     'H-Nx'; 'H-Ny'; 'H-Nz';
%     'HGradD-CurvEQ'; 'HGradM-CurvEQ';
%     %     'GradD-Curv'; 'GradM-Curv';
%     %     'GradDFil-Curv'; 'GradMFil-Curv';
%     %     'Inten-RefEQ'; 'IntenEQ-RefEQ';
%     };

sm_list = {
     'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
     'int-norm_x'; 'int-norm_y'; 'int-norm_z';
     'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
     'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
    };

% sm_list = {...
%     'Inten-Ref'; 'S-Ref'; 'V-Ref'; 'IntenEQ-Ref'; 'SEQ-Ref'; 'VEQ-Ref';
%     'Inten-RefEQ'; 'S-RefEQ'; 'V-RefEQ'; 'IntenEQ-RefEQ'; 'SEQ-RefEQ'; 'VEQ-RefEQ';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     'IntenEQ-Nx'; 'IntenEQ-Ny'; 'IntenEQ-Nz';
%     'GradD-Curv'; 'GradM-Curv'; 'GradDEQ-Curv'; 'GradMEQ-Curv';
%     };

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

% err_list = {...
%     'heading';
%     };

sm_type = 'NMI';

for di=6:7
    % di=5;
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    computeSMTrend(img_list, points, para);
    
    % para.x0 = gradient_descent_search_grid (para.x0, img_list, points, para);
    % mi_cost_2d(para.x0, img_list, points, para,'test_rst');
end