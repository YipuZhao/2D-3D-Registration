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
%     'Inten-Ref'; 'IntenEQ-Ref'; 'Y-Ref'; 'H-Ref';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     %     'IntenEQ-Nx'; 'IntenEQ-Ny'; 'IntenEQ-Nz';
%     %     'H-Nx'; 'H-Ny'; 'H-Nz';
%     'HGradD-CurvEQ'; 'HGradM-CurvEQ';
%     %     'GradD-Curv'; 'GradM-Curv';
%     %     'GradDFil-Curv'; 'GradMFil-Curv';
%     %     'Inten-RefEQ'; 'IntenEQ-RefEQ';
%     };
sm_list = {...
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
% sm_type = 'MI';

bias_step_num = 20;
skip_frame_num = 20;
sm_min = 0;
sm_max = 0.01;
step_num = 20;
sm_num = length(sm_list);
err_num = length(err_list);
dir_cluster = ['..\..\Data\GT_reorg\cluster\'];
% dir_cluster = ['..\..\Data\kitti\cluster\'];
% dir_cluster = ['D:\\Copy\cluster_atl_h\'];
load([dir_cluster 'cluster.mat']);
load([dir_cluster 'data.mat']);

%
frame_sum = 0;
for di = 1:6
    % load para
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    nbins = round(256/para.bin_fraction);
%     tmp_x0 = para.x0;
    % load data
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    frame_num = length(img_list);
    %
    for img_idx = 1:skip_frame_num:frame_num
        if sm_num > 1
            % query the cluster for current image
            for clu_idx = 1:length(cluster)
                query_res = find(cluster{clu_idx}.patch_idx == img_idx + frame_sum);
                if ~isempty(query_res)
                    % return clu_idx
                    break ;
                end
            end
            %
            if clu_idx >= 1 && clu_idx <= length(cluster)
                w{1} = cluster{clu_idx}.w;
            else
                w{1} = ones(1, sm_num);
                w{1} = w{1} / sum(w{1});
            end
        else
            %
            w{1} = 1;
        end
        %
        image = img_list{img_idx};
        frame_no = image.frame_no
        
        %%
%         if di ~= 8
%             continue ;
%         end
%         if frame_no ~= 0 && frame_no ~= 60 && frame_no ~= 120
%             continue ;
%         end
if di ~= 3 && di ~= 4
            continue ;
        end
        if frame_no ~= 4310 && frame_no ~= 4940 && frame_no ~= 4980
            continue ;
        end
        
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
        [gm_img, ~] = imgradient(image.img_attribute{4} );
        %         image.img_attribute{5} = im2double(gd_img);
        image.img_attribute{5} = im2double(gm_img);
        %
        for eno=1:err_num
            err_fname = ['error_' err_list{eno} '\'];
            if ~exist([para.dir_output, dir_frame, err_fname], 'dir')
                mkdir([para.dir_output, dir_frame, err_fname]);
            end
            for iter_no = 1+step_num - bias_step_num : 1 : 1+step_num + bias_step_num
                % for iter_no = 1+step_num - bias_step_num
%                 if iter_no == 1+step_num
%                     continue;
%                 end
                % add error to initial parameters
                [tmp_x0, t, R, inv_R, err_val] = addError(para.err_list{eno}, para.x0, iter_no, step_num);
                err_val
                % get corresponding points
                [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
                 % for kitti data
                if strcmp(para.camera_type, 'kitti')
                    [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                    % for GT data
                else
                    [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points);
                end
%                 [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                reg_img_list{1} = image;
                reg_points = points(points_id, :);
                % perform 2D-3D registration
                %                 figure;
                %                 imagesc(image.image);
                %                 hold on
                %                 scatter(u, v, 3*ones(1,length(u)), points(points_id, 5), 'filled');
                %                 file_name = sprintf('Original_%s_%d.jpg', para.err_list{eno}, iter_no-(1+step_num));
                %                 set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
                %                 saveas(gcf,[para.dir_output, dir_frame, err_fname, file_name]);
                %
                %                 para.SM_type = 'MI'
                %                 mi_x0 = gradient_descent_search_s2 (tmp_x0, reg_img_list, reg_points, para, w);
                % %                 nmi_x0 = fminsearchbnd(...
                % %                     @errorEstimate, tmp_x0, ...
                % %                     tmp_x0 - [deg2rad(0.1), deg2rad(0.1), deg2rad(0.1), 0.025, 0.025, 0.025] * bias_step_num, ...
                % %                     tmp_x0 + [deg2rad(0.1), deg2rad(0.1), deg2rad(0.1), 0.025, 0.025, 0.025] * bias_step_num, ...
                % %                     optimset('PlotFcns', @optimplotfval, 'MaxIter', 3000));
                %
                %                 % visualize results
                %                 [t, R, inv_R, err_val] = addError(para.err_list{1}, mi_x0, step_num+1, step_num);
                %                 [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
                %                 [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                %                 figure;
                %                 imagesc(image.image);
                %                 hold on
                %                 scatter(u, v, 3*ones(1,length(u)), points(points_id, 5), 'filled');
                %                 file_name = sprintf('MI_%s_%d.jpg', para.err_list{eno}, iter_no-(1+step_num));
                %                 set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
                %                 saveas(gcf,[para.dir_output, dir_frame, err_fname, file_name]);
                % %                 savefig(gcf, [para.dir_output, dir_frame, file_name], 'compact');
                %                 file_name = sprintf('MI_%s_%d.mat', para.err_list{eno}, iter_no-(1+step_num));
                %                 save([para.dir_output, dir_frame, err_fname, file_name], 'mi_x0');
                %                 % restore
                %                 %         para.x0 = ori_x0;
                %                 close all;
                
                %%
                para.SM_type = 'MI';
                tic
                mi_x0 = gradient_descent_search_s2 (tmp_x0, reg_img_list, reg_points, para, w);
                toc
                % visualize results
                [mi_x0, t, R, inv_R, err_val] = addError(para.err_list{1}, mi_x0, step_num+1, step_num);
                [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
                % for kitti data
                if strcmp(para.camera_type, 'kitti')
                    [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                    % for GT data
                else
                    [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points);
                end
%                 [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                figure;
                imagesc(image.image);
                hold on
                scatter(u, v, 3*ones(1,length(u)), points(points_id, 5), 'filled');
                file_name = sprintf('%s_%s_%d.jpg', para.SM_type, para.err_list{eno}, iter_no);
                set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
                saveas(gcf,[para.dir_output, dir_frame, err_fname, file_name]);
                %                 savefig(gcf, [para.dir_output, dir_frame, file_name], 'compact');
                file_name = sprintf('%s_%s_%d.mat', para.SM_type, para.err_list{eno}, iter_no);
                save([para.dir_output, dir_frame, err_fname, file_name], 'mi_x0');
                
                %%
                para.SM_type = 'NMI';
                tic
                mi_x0 = gradient_descent_search_s2 (tmp_x0, reg_img_list, reg_points, para, w);
                toc
                % visualize results
                [mi_x0, t, R, inv_R, err_val] = addError(para.err_list{1}, mi_x0, step_num+1, step_num);
                [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
                % for kitti data
                if strcmp(para.camera_type, 'kitti')
                    [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                    % for GT data
                else
                    [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points);
                end
%                 [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
                figure;
                imagesc(image.image);
                hold on
                scatter(u, v, 3*ones(1,length(u)), points(points_id, 5), 'filled');
                file_name = sprintf('%s_%s_%d.jpg', para.SM_type, para.err_list{eno}, iter_no);
                set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
                saveas(gcf,[para.dir_output, dir_frame, err_fname, file_name]);
                %                 savefig(gcf, [para.dir_output, dir_frame, file_name], 'compact');
                file_name = sprintf('%s_%s_%d.mat', para.SM_type, para.err_list{eno}, iter_no);
                save([para.dir_output, dir_frame, err_fname, file_name], 'mi_x0');
                % restore
                %         para.x0 = ori_x0;
                close all;
            end
        end
    end
    frame_sum = frame_sum + frame_num;
end