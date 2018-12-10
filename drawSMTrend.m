clear all;
close all;

global step_num

%
% data_dir = '..\..\data\kitti\similarity\kitti0093_output\frame_160\';
data_dir = '..\..\data\GT_reorg\similarity\site3_output\frame_4370\';
% i=2;j=2;
% err_no = 1:6;
err_shift = 0;
step_num = 20;%20;

err_fname = {
    'error_heading';
    'error_pitching';
    'error_rolling';
    'error_shift_x';
    'error_shift_y';
    'error_shift_z';
    };

error_desc = {
    'Rotation - h';
    'Rotation - p';
    'Rotation - r';
    'Tansition - x';
    'Tansition - y';
    'Tansition - z';
    };

% sm_list = {...
%     'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%     'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%     'grad_{mag}-curv'; 'grad_{dir}-curv';
%     };
sm_list = {
    'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
    'int-norm_x'; 'int-norm_y'; 'int-norm_z';
    'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
    'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
    };

cur_sm_arr = zeros(2*step_num+1,13);

for err_no = 1:6
    
    files = dir( fullfile([data_dir err_fname{err_no}], 'SM_frame_*.mat') );   %# list all *.xyz files
    files = {files.name}';                      %'# file names
    
    fname = fullfile([data_dir err_fname{err_no}], files{1});
    tmp_sm = load(fname);
    
    %     cur_sm_arr = cur_sm_arr + tmp_sm.cur_sm_arr;
    cur_sm_arr = tmp_sm.cur_sm_arr;
    % w = [0.0791301931037251,0.162277813038427,0.0225820842847348,0.154326668172485,0.146673881872193,0.170789176912302,0.0975050574285123,0.112443103087265,0.0542720221003555];
    %  w = [0,0.3,0,0.4,0,0,0.3,0,0];
    w = [0.120106417416860,0.120106417416860,0.0765677251157847,0.0865927233995035,0.0640974263765096,0.0431626496836985,0.120106417416860,0.0944840840102062,0.0818691398989107,0.0706517357252729,0.0162519251807352,0.0569028479349688,0.0491004904238286];
    comb_sm_arr = w * cur_sm_arr';
    
    h = figure;
    %     set(h,'OuterPosition',[2 2 8 4]);
    %     h.PaperUnits = 'inches';
    %     set(h,'PaperUnits','inches')
    %     set(h, 'Position', [0 0 60 20])
    min_val = min(cur_sm_arr(:, :), [], 1);
    if err_shift < 0
        plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1)+err_shift, 1), '-o');
        hold on;
        plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1)+err_shift, 1), '-x');
        plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1)+err_shift, 1), '-.');
        %
        plot(-step_num-err_shift : step_num, comb_sm_arr(1:end+err_shift) - min(comb_sm_arr), 'r-', 'LineWidth',3);
    else
        plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1)-err_shift, 1), '-o');
        hold on;
        plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1)-err_shift, 1), '-x');
        plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1)-err_shift, 1), '-.');
        %
        plot(-step_num : step_num-err_shift, comb_sm_arr(1+err_shift:end) - min(comb_sm_arr), 'r-', 'LineWidth',3);
    end
    % ylim([0, 0.08]);
    legend(sm_list);
    % xlabel(['Registration Error (' error_desc{err_no} ')']);
    %     xlabel('Registration error (rotation - 0.1 deg/step, transition - 0.025 m/step)');
    xlabel('Registration error');
    ylabel('NMI');
    
    for sno=1:length(sm_list)
        sc(sno) = scoreEstimation(cur_sm_arr(:,sno));
    end
    sc
    
    
end
% for pno = 1:length(sm_list)
%     figure;
%     min_val = min(cur_sm_arr(:, :), [], 1);
%     if err_shift < 0
%         plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1)+err_shift, 1), '-o');
%         hold on;
%         plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 5:7) - repmat(min_val(5:7), size(cur_sm_arr, 1)+err_shift, 1), '-x');
%         plot(-step_num-err_shift : step_num, cur_sm_arr(1:end+err_shift, 8:9) - repmat(min_val(8:9), size(cur_sm_arr, 1)+err_shift, 1), '-.');
%     else
%         %     plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 1:4, i, j) - repmat(min_val(1:4), size(cur_sm_arr, 1)-err_shift, 1), '-o');
%         %     hold on;
%         %     plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 5:7, i, j) - repmat(min_val(5:7), size(cur_sm_arr, 1)-err_shift, 1), '-x');
%         %     plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, 8:9, i, j) - repmat(min_val(8:9), size(cur_sm_arr, 1)-err_shift, 1), '-.');
%         plot(-step_num : step_num-err_shift, cur_sm_arr(1+err_shift:end, pno) - repmat(min_val(pno), size(cur_sm_arr, 1)-err_shift, 1), '-o' );
%         ylim([0, 0.08]);
%     end
%     legend(sm_list{pno});
%     % xlabel(['Registration Error (' error_desc{err_no} ')']);
%     xlabel('Registration error (rotation - 0.1 deg/step, transition - 0.025 m/step)');
%     ylabel('Average NMI on 6 types of registration errors');
% end
