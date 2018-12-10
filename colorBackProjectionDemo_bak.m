close all;
clear all;
clc
data_list = {...
    'site1';
    'site2';
    'site3';
    'site4';
    %
    'site2_run2';
    'site3_run2';
    %
    'site2_run3';
    'site3_run3';
    %     'run3_v';
    %     'Pandey';
    'kitti0002';
    'kitti0093';
    'kitti0104';
    'kitti0106';
    %
    'I475N_150529';
    'I475S_150529';
    'I475N_150326';
    'I475S_150326';
    };

% sm_list = {
%      'int-ref'; 'int_{hist}-ref'; 'int_{shadow}-ref'; 'hue-ref';
%      'int-norm_x'; 'int-norm_y'; 'int-norm_z';
%      'int_{hist}-norm_x'; 'int_{hist}-norm_y'; 'int_{hist}-norm_z';
%      'grad^{hue}_{mag}-curv'; 'grad^{hue}_{mag}-grad^{ref}_{mag}'; 'grad^{hue}_{mag}-grad^{comb}_{mag}';
%     };
sm_list = {
    'int-int'; 
    'int_{hist}-int_{hist}'; 
    'int_{shadow}-int_{shadow}'; 
    'hue-hue';
    };

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

step_num = 20;

for di=6
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    points(:, 13:16) = -1;
    
    nbins = round(256/para.bin_fraction);
    sm_num = length(para.sm_list);
    frame_num = length(img_list);
    tmp_x0 = para.x0;
    for img_idx = 1:frame_num-1
        prior_image = img_list{img_idx};
        [tmp_x0, t, R, inv_R, ~] = addError(para.err_list{1}, para.x0, 1+step_num, step_num);
        [u, v, points_id] = imageProjection_upd(points, prior_image, para, t, R, inv_R);
        [u, v, points_id] = occludedPointsRemoval(u, v, points_id, prior_image, points);
        % project rgb info from image to point cloud
        prior_int_img = rgb2gray(prior_image.image);
        prior_inteq_img = adapthisteq(prior_int_img);
        tmp_img = shadowremoval(im2double(prior_image.image), 'ycbcr');
        prior_y_img = tmp_img(:,:,1);
        tmp_img = rgb2hsv(double(prior_image.image));
        prior_h_img = tmp_img(:,:,1);
        %
        prior_int_arr = im2double(prior_int_img);
        prior_inteq_arr = im2double(prior_inteq_img);
        prior_y_arr = im2double(prior_y_img);
        prior_h_arr = im2double(prior_h_img);
        %
        prior_int_arr = prior_int_arr(:)';
        points(points_id, 13) = prior_int_arr(v + (u-1)*para.cam.height)';
        prior_inteq_arr = prior_inteq_arr(:)';
        points(points_id, 14) = prior_inteq_arr(v + (u-1)*para.cam.height)';
        prior_y_arr = prior_y_arr(:)';
        points(points_id, 15) = prior_y_arr(v + (u-1)*para.cam.height)';
        prior_h_arr = prior_h_arr(:)';
        points(points_id, 16) = prior_h_arr(v + (u-1)*para.cam.height)';
        %
        cur_points = points(points_id, :);
        
        figure;
        subplot(1,2,1)
        imagesc(prior_image.image);
        hold on
        scatter(u, v, 3*ones(1,length(u)), cur_points(:, 6), 'filled');
        subplot(1,2,2)
        scatter3(cur_points(:, 2), cur_points(:, 3), cur_points(:, 4), ones(1, length(cur_points)), cur_points(:, 13));
        axis equal
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 24])
        
        %%
        image = img_list{img_idx+1};
        frame_no = image.frame_no
        dir_frame = ['frame_' num2str(frame_no) '\'];
        mkdir([para.dir_output dir_frame]);
        
        file_name = sprintf('Color_projection_frame_%04d', frame_no);
        saveas(gcf, [para.dir_output, dir_frame, file_name]);
        close all;
        
        R_w2v = makeRotationMatrix(image.prh);
        inv_R_w2v = inv(R_w2v);
        %
        int_img = rgb2gray(image.image);
        inteq_img = adapthisteq(int_img);
        tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        y_img = tmp_img(:,:,1);
        tmp_img = rgb2hsv(double(image.image));
        h_img = tmp_img(:,:,1);
        % plot trend of mi between rgb info
        for eno = 1:length(para.err_list)
            err_frame = ['error_' para.err_list{eno} '\'];
            mkdir([para.dir_output dir_frame err_frame]);
            for iter_no = 1:1+2*step_num
                %% add error to registration parameters
                [~, t, R, inv_R, ~] = addError(para.err_list{eno}, para.x0, iter_no, step_num);
                [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
%                 [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                %% keep points with rgb info
                points_rgb = points(points_id, :);
                rgb_vld_idx = find(points_rgb(:, 13) > 0);
                points_rgb = points_rgb(rgb_vld_idx, :);
                u = u(rgb_vld_idx);
                v = v(rgb_vld_idx);
                %
                pcd_int_arr = points_rgb(:, 13);
                pcd_inteq_arr = points_rgb(:, 14);
                pcd_y_arr = points_rgb(:, 15);
                pcd_h_arr = points_rgb(:, 16);
                for sn = 1 : sm_num
                    % compute sm for each pair of attributes
                    switch para.sm_list{sn}
                        case 'int-int'
                            img_attr = im2double(int_img);
                            pcd_attr = double(pcd_int_arr);
                        case 'int_{hist}-int_{hist}'
                            img_attr = im2double(inteq_img);
                            pcd_attr = double(pcd_inteq_arr);
                        case 'int_{shadow}-int_{shadow}'
                            img_attr = im2double(y_img);
                            pcd_attr = double(pcd_y_arr);
                        case 'hue-hue'
                            img_attr = im2double(h_img);
                            pcd_attr = double(pcd_h_arr);
                    end
                    %
                    img_attr = img_attr(:)';
                    img_attr = img_attr(v + (u-1)*para.cam.height)';
                    [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
                    [sm_arr(iter_no, sn), ~, ~, ~] = get_NMI(MI_prob, nbins);
                end
            end
            %%
            figure;
            min_val = min(sm_arr(:, :), [], 1);
            plot(-step_num : step_num, sm_arr(:, 1:4) - repmat(min_val(1:4), size(sm_arr, 1), 1), '-o');
            legend(para.sm_list)
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
            file_name = sprintf('Color_Trend_frame_%04d_%s', frame_no, para.err_list{eno});
            saveas(gcf, [para.dir_output, dir_frame, err_frame, file_name]);
            close all;
        end
    end
end