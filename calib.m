clear all
close all
global err_num
% global reg_img_list
% global reg_points
% global para
% global w
global step_num
global sm_num

%%

data_list = {
    'site1';    'site2';    'site3';    'site4';
    'site1_v';    'site2_v';    'site3_v';    'site4_v';
    'kitti0002';    'kitti0093';    'kitti0104';    'kitti0106';
    'change_ref0_h'; 'change_ref0_v'; 'change_tilt0_h'; 'change_tilt0_v';
'I475N_141112_h'; 'I475N_141112_v';
'I475N_150130_h'; 'I475N_150130_v';
'I475N_150529_h'; 'I475N_150529_v';
'I475N_150701_h'; 'I475N_150701_v';
'calib_150326_h'; 'calib_150326_v';
    };

sm_list = {
     'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
     'int-norm_x'; 'int-norm_y'; 'int-norm_z';
     'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
     'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
    };
% sm_list = {
%     'int-ref';
%     };

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

sm_type = 'NMI';

skip_frame_num = 5;%15;
sm_min = 0;
sm_max = 0.01;
step_num = 20;
sm_num = length(sm_list);
err_num = 6;
% dir_cluster = ['C:\Users\yzhao347\Copy\cluster_test\'];
dir_cluster = ['D:\\Copy\cluster_atl_v\'];
load([dir_cluster 'cluster.mat']);
load([dir_cluster 'data.mat']);

% para.x0 = [-0.4765748086480437,	-0.10868538971935367,	0.360704670619655,...
%     -0.1049723896187755,	-0.020188580624356477,	-0.3469560326690064
% ];
%
rno = 1;
reg_points = [];
for di = 8
    % load para
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    nbins = round(256/para.bin_fraction);
    tmp_x0 = para.x0;
    % load data
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    frame_num = length(img_list);
    %
    
    for img_idx = 1:skip_frame_num:frame_num
        image = img_list{img_idx};
        frame_no = image.frame_no
        dir_frame = ['frame_' num2str(frame_no) '\'];
        % estimate 2D attributes
        image.img_attribute{1} = im2double(rgb2gray(image.image));
        image.img_attribute{2} = im2double(adapthisteq(image.img_attribute{1}));
        % image - illumination inv
        tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        image.img_attribute{3} = im2double(tmp_img(:,:,1));
        %     cb_img = tmp_img(:,:,2);
        %     cr_img = tmp_img(:,:,3);
        tmp_img = rgb2hsv(double(image.image));
        image.img_attribute{4} = im2double(tmp_img(:,:,1));
        %     s_img = tmp_img(:,:,2);
        %     v_img = tmp_img(:,:,3);
        % image - gradient
        % [~, gd_img_int] = imgradient(inten_img);
        % [~, gd_img_inteq] = imgradient(inten_img_eq);
        [gm_img, gd_img] = imgradient(image.img_attribute{4} );
        image.img_attribute{5} = im2double(gm_img);
        %         image.img_attribute{6} = im2double(gm_img);
        %
        [~, t, R, inv_R, err_val] = addError(para.err_list{1}, tmp_x0, step_num+1, step_num);
        % get corresponding points
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        reg_img_list{rno} = image;
        reg_points = [reg_points; points(points_id, :)];
        % get scene label
                if img_list{img_idx}.label == -1 || sm_num == 1
                    w{rno} = ones(1, sm_num);
                    w{rno} = w{rno} / sum(w{rno});
                else
                    w{rno} = cluster{img_list{img_idx}.label}.w;
                end
%         w{rno} = ones(1, sm_num);
% w{rno} = [0.380779354867005,0.334985625622913,0.00411299275633137,0.160203562012357,0.000474060560658990,1.03066049044463e-05,0.0733103561542996,3.48838244033863e-05,0.000969274365406947,0.0204123504227138,0.0177457287967988,0.00696115309997825,3.50912229108639e-07];
% w{rno} = w{rno} / sum(w{rno});
        
        rno = rno + 1;
    end
    
end

nmi_x0 = gradient_descent_search_s2 (para.x0, reg_img_list, reg_points, para, w);

%     file_name = sprintf('Calib.mat');
%     mkdir(para.dir_output);
%     save([para.dir_output, file_name], 'nmi_x0');
%                 nmi_x0 = fminsearchbnd(...
%                     @errorEstimate, tmp_x0, ...
%                     tmp_x0 - [deg2rad(0.1), deg2rad(0.1), deg2rad(0.1), 0.025, 0.025, 0.025] * bias_step_num, ...
%                     tmp_x0 + [deg2rad(0.1), deg2rad(0.1), deg2rad(0.1), 0.025, 0.025, 0.025] * bias_step_num, ...
%                     optimset('PlotFcns', @optimplotfval, 'MaxIter', 3000));
for di = 8
    % load para
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    mkdir(para.dir_output);
    % load data
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    frame_num = length(img_list);
    %
    % visualize results
    for img_idx = 1:skip_frame_num:frame_num
        image = img_list{img_idx};
        frame_no = image.frame_no
        [~, t, R, inv_R, err_val] = addError(para.err_list{1}, nmi_x0, step_num+1, step_num);
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        figure;
        imagesc(image.image);
        hold on
        scatter(u, v, 3*ones(1,length(u)), points(points_id, 5), 'filled');
        file_name = sprintf('Calib_frame_%d.jpg', frame_no);
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        saveas(gcf,[para.dir_output, file_name]);
        % restore
        %         para.x0 = ori_x0;
        close all;
    end
end