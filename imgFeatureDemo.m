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
    'I475N_150326_h';
    'I475N_150430_h';
    'I475N_150529_h';
    %     'I475N_150701_h';
    };

% sm_list = {
%      'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%      'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%      'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
%      'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
%     };
sm_list = {
    'int-ref';
    };

err_list = {...
    'heading';
    'rolling';
    'pitching';
    'shift_x';
    'shift_y';
    'shift_z';
    };

sm_type = 'NMI';

skip_frame_num = 5;
sm_min = 0;
sm_max = 0.01;
step_num = 20;
sm_num = length(sm_list);
err_num = 6;

rno = 1;
reg_points = [];
for di = 17 : 19
    % load para
    para{di-16} = loadPara(data_list{di}, sm_list, err_list, sm_type);
    nbins = round(256/para{di-16}.bin_fraction);
    tmp_x0 = para{di-16}.x0;
    % load data
    [points{di-16}, img_list{di-16}] = loadData(para{di-16}.dir_name, para{di-16}.dir_image_name, para{di-16});
    %     frame_num = length(img_list);
    %
end

[optimizer, metric] = imregconfig('monomodal');
for img_idx = 1:skip_frame_num:100
    %
    features_old = [];
    for di=17:19
        image = img_list{di-16}{img_idx};
        frame_no = image.frame_no
        %         dir_frame = ['frame_' num2str(frame_no) '\'];
        if ~exist([para{di-16}.dir_output], 'dir')
            mkdir([para{di-16}.dir_output]);
        end
        %
        %         rgb_img = image.image;
        gray_img = rgb2gray(image.image);
        keypts = detectSURFFeatures(gray_img);
        [features, vld_keypts] = extractFeatures(gray_img, keypts);
        figure;
        imagesc(gray_img);
        hold on;
        plot(vld_keypts);
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        file_name = sprintf('ImgFeature_imgidx_%d.jpg', img_idx);
        saveas(gcf, [para{di-16}.dir_output, file_name]);
        
        if ~isempty(features_old)
            [gray_img_reg] = imregister(gray_img,gray_img_old,'rigid',optimizer,metric);
            figure;
            subplot(1,2,1);
            imshow(imfuse(gray_img_old,gray_img));
            subplot(1,2,2);
            imshow(imfuse(gray_img_old,gray_img_reg));
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 24 9])
            file_name = sprintf('Recover_MI_imgidx_%d.jpg', img_idx);
            saveas(gcf, [para{di-16}.dir_output, file_name]);
            %
            index_pairs = matchFeatures(features_old, features);
            mat_keypts_old = vld_keypts_old(index_pairs(:,1));
            mat_keypts = vld_keypts(index_pairs(:,2));
            figure;
            showMatchedFeatures(gray_img_old, gray_img, mat_keypts_old, mat_keypts);
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
            file_name = sprintf('Matched_imgidx_%d.jpg', img_idx);
            saveas(gcf, [para{di-16}.dir_output, file_name]);
            %
            [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(mat_keypts,mat_keypts_old,'similarity');
            outputView = imref2d(size(gray_img_old));
            Ir = imwarp(gray_img,tform,'OutputView',outputView);
            figure;
            subplot(1,2,1);
            imshow(imfuse(gray_img_old,gray_img));
            subplot(1,2,2);
            imshow(imfuse(gray_img_old,Ir));
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 24 9])
            file_name = sprintf('Recover_SURF_imgidx_%d.jpg', img_idx);
            saveas(gcf, [para{di-16}.dir_output, file_name]);
        end
        
        features_old = features;
        vld_keypts_old = vld_keypts;
        gray_img_old = gray_img;
        %         rgb_img_old = rgb_img;
        
        close all
    end
end