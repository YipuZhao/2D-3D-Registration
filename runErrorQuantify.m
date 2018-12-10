close all;
clear all;
clc
data_list = {...
    'site1';
    'site2';
    'site3';
    'site4';
    'I475N_150326';
    'I475S_150326';
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
sm_list = {...
    'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
    'int-norm_x'; 'int-norm_y'; 'int-norm_z';
    'grad_{mag}-curv'; 'grad_{dir}-curv';
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

approach_list = { 
    'MI';
    'NMI';
    'Optimise';
    };

% approach_list = {
%     0;
%     1;
%     };

for di=9
    % di=5;
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    dist_buf = projectionErrorQuantify(img_list, points, para, approach_list);
    
    % para.x0 = gradient_descent_search_grid (para.x0, img_list, points, para);
    % mi_cost_2d(para.x0, img_list, points, para,'test_rst');
end

% err_title = {
%     'Initial error on rolling - 0.1 deg/step';
%     'Initial error on heading - 0.1 deg/step';
%     'Initial error on pitching - 0.1 deg/step';
%     'Initial error on x transition - 0.025 m/step';
%     'Initial error on x transition - 0.025 m/step';
%     'Initial error on x transition - 0.025 m/step';
%     };

for ano=1:3
    
    figure;
    total_dist_mat = [];
    for eno=1:6
        cur_dist_mat(:,:) = dist_buf(ano, eno, :, :)
        
        %         for iter_no=1:11
        valid_idx = find(cur_dist_mat(1,:)>0);
        %             cur_dist_arr = cur_dist_mat(iter_no,valid_idx);
        %             mean(cur_dist_arr);
        %         end
        total_dist_mat = [total_dist_mat, cur_dist_mat(:,valid_idx)]
    end
    boxplot(total_dist_mat(1:2:end, :)', 'plotstyle', 'traditional', 'labels', -10:10, 'symbol', '');
    
%     xlabel('Initial condition error (rotation - 0.1 deg/step, transition - 0.025 m/step)')
    xlabel('Initial condition error')
    ylabel('Average projection error on 2D image')
%     legend(approach_list);
    save([approach_list{ano} '_pixel_bias'], 'total_dist_mat');
%     ylim([0 20])
end

