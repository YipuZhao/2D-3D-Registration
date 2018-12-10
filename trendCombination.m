clear all
close all
global err_num
global sm_image_buf
global step_num
global sm_num

%%

% data_list = {...
%     'site1';
%     'site2';
%     'site3';
%     'site4';
%     'site1_v';
%     'site2_v';
%     'site3_v';
%     'site4_v';
%     %     'run3_v';
%     %     'Pandey';
%     'kitti0002';
%     'kitti0093';
%     'kitti0104';
%     'kitti0106';
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

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

sm_min = 0;
sm_max = 0.01;
step_num = 20;
sm_num = length(sm_list);
err_num = length(err_list);
%
dir_output = ['..\..\data\kitti\cluster\'];
load([dir_output 'cluster.mat']);
load([dir_output 'data.mat']);

%%
for i=1:length(cluster)
    w = weightOptimization(cluster{i})
    %
    cluster{i}.w = w;
    cur_sm_arr(:, :) = sum(cluster{i}.trend_avg(:, :, :), 1) / (cluster{i}.count*err_num);
    cluster{i}.trend_comb = cluster{i}.w * cur_sm_arr(:, :)';
    %
    close all;
    figure;
    min_val = min(cur_sm_arr(:, :), [], 1);
    plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
    hold on;
    plot(-step_num : step_num, cur_sm_arr(:, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1), 1), '-x');
    plot(-step_num : step_num, cur_sm_arr(:, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1), 1), '-.');
    plot(-step_num : step_num, cluster{i}.trend_comb - min(cluster{i}.trend_comb), 'r-', 'LineWidth',3);
    %     ylim([sm_min sm_max]);
    ylim([0, 0.035]);
    legend([sm_list; 'Combination']);
    title(['Urban scene category ' num2str(i) ' patch number ' num2str(cluster{i}.count)]);
    %     title(['average trend of cluster ' num2str(i) ' patch number ' num2str(cluster{i}.count)]);
    xlabel('Registration error (rotation - 0.1 deg/step, transition - 0.025 m/step)');
    ylabel('Average NMI on 6 types of registration errors');
    %     dir_cluster = [dir_output 'cluster_' num2str(i) '\'];
    file_name = sprintf('Trend_combination_%d.jpg', i);
    saveas(gcf, [dir_output file_name]);
end
save([dir_output 'cluster.mat'], 'cluster');
