clear all;
close all;

global step_num

%% pre-defined parameters
% sm_min = 0.8;
% sm_max = 1.4;
sm_min = 0;
sm_max = 0.15;
step_num = 20;
% adj_step = 0;%4;%2;
points_thres = 1000;

%
score_thres = -0.1;%0.04;
dist_thres = -0.1;%0.02;
% upbound_side_score = 0.2;
% alpha = 5;
% base_w = 50
% w = [base_w + 1 : 1 : base_w + step_num, base_w + step_num : -1 : base_w + 1];
% w = w / sum(w) * 100;

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
% err_list = {...
%     'heading';
%     };

sm_type = 'NMI';

%% load trend data for each frame
k = 1;
for di=7:9
    % di = 5;
    %     dir_trend = ['..\..\data\kitti\similarity\' data_list{di} '_output\'];
    if di <= 4
        dir_trend = ['..\..\data\GT_reorg\similarity\' data_list{di} '_output\'];
    elseif di<=6
        dir_trend = ['..\..\data\Hwy_reorg\similarity\' data_list{di} '_output\'];
    else
        dir_trend = ['..\..\data\kitti\similarity\' data_list{di} '_output\'];
    end
    % dir_trend = ['.\data\' data_list{di} '_output\'];
    
    para = loadPara(data_list{di}, sm_list, err_list, sm_type);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    %     grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
    %     grid_x(end) = para.ROI_width(2)+1;
    %     grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
    %     grid_y(end) = para.ROI_height(2)+1;
    
    frame_num = length(img_list);
    sm_num = length(para.sm_list);
    err_num = length(para.err_list);
    
    % sm_image_buf = zeros(frame_num, 2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
    for img_idx = 1:frame_num
        %
        image = img_list{img_idx};
        frame_no = image.frame_no
        R_w2v = makeRotationMatrix(image.prh);
        inv_R_w2v = inv(R_w2v);
        
        %         tmp_img = rgb2gray(image.image);
        %         inten_img = tmp_img(para.ROI_height(1):para.ROI_height(2), para.ROI_width(1):para.ROI_width(2));
        R = makeRotationMatrix(para.x0(4:6));
        inv_R = inv(R);
        t = para.x0(1:3)';
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        %         [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
        % for kitti data
        if strcmp(para.camera_type, 'kitti')
            [u, v, points_id] = occludedPointsRemoval(u, v, points_id, image, points);
            % for GT data
        else
            [u, v, points_id] = occludedPointsRemoval_upd(u, v, points_id, image, points);
        end
        
        %         inten_img = rgb2gray(image.image);
        %         inten_img_eq = adapthisteq(inten_img);
        %         % image - illumination inv
        %         tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        %         y_img = tmp_img(:,:,1);
        %         tmp_img = rgb2hsv(double(image.image));
        %         h_img = tmp_img(:,:,1);
        %         % image - gradient
        %         [gm_img_hue, gd_img_hue] = imgradient(h_img);
        
        %         %         ref_arr = points(points_id(:), 6);
        %         x_pcd_arr = double(points(points_id,2));
        %         y_pcd_arr = double(points(points_id,3));
        %         z_pcd_arr = double(points(points_id,4));
        ref_pcd_arr = double(points(points_id,5)); % refc
        %         ref_pcd_arr_eq = double(points(points_id,6));
        % ref_pcd_arr_eq = histeq(ref_pcd_arr);
        %         % point cloud - normal
        %         nx_pcd_arr = double(points(points_id,7));
        %         ny_pcd_arr = double(points(points_id,8));
        %         nz_pcd_arr = double(points(points_id,9));
        %         % convert normal from wc to vc
        %         norm_VC = inv_R_w2v * [nx_pcd_arr, ny_pcd_arr, nz_pcd_arr]';
        %         nx_pcd_arr = norm_VC(1, :)';
        %         ny_pcd_arr = norm_VC(2, :)';
        %         nz_pcd_arr = norm_VC(3, :)';
        %         % point cloud - curvature
        %         curv_pcd_arr = double(points(points_id,10)); % cur
        %         curv_pcd_arr_eq = histeq(curv_pcd_arr);
        %         % point cloud - edge
        %         edge_pcd_arr = double(points(points_id,11));
        %         max_curv = max(curv_pcd_arr_eq);
        %         curv_pcd_arr_eq = curv_pcd_arr_eq + edge_pcd_arr * max_curv;
        
        dir_frame = ['frame_' num2str(frame_no) '\'];
        %         for i = 1:para.grid_row_num
        %             for j = 1:para.grid_col_num
        k_stop = false;
        %
        xbias = -para.ROI_width(1)+1;
        ybias = -para.ROI_height(1)+1;
        % points for grid[i][j]
        %                 grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
        % remove 2D patches without enough 3D points projected
        if length(ref_pcd_arr(:)) < points_thres
            continue ;
        end
        sm_image_buf{k}.points(:, :) = [ u', v', ref_pcd_arr ];
        %             u', ...
        %             v', ...
        %             x_pcd_arr(:), ...
        %             y_pcd_arr(:), ...
        %             z_pcd_arr(:), ...
        %             ref_pcd_arr_eq, ...
        %             nx_pcd_arr(:),...
        %             ny_pcd_arr(:),...
        %             nz_pcd_arr(:),...
        %             curv_pcd_arr_eq(:)
        %             ];
        %
        sm_image_buf{k}.image = image;
        %
        %         sm_image_buf{k}.image.img_attribute{1} = inten_img;
        %         sm_image_buf{k}.image.img_attribute{2} = inten_img_eq;
        %         sm_image_buf{k}.image.img_attribute{3} = y_img;
        %         sm_image_buf{k}.image.img_attribute{4} = h_img;
        %         sm_image_buf{k}.image.img_attribute{5} = gd_img_hue;
        %         sm_image_buf{k}.image.img_attribute{6} = gm_img_hue;
        %
        for eno = 1:err_num
            err_frame = ['error_' para.err_list{eno} '\'];
            %
            file_name = sprintf('SM_frame_%04d_%s', frame_no, para.err_list{eno});
            load([dir_trend, dir_frame, err_frame, file_name '.mat']);
            %
            if min(min(cur_sm_arr(:, :))) == 0
                k_stop = true;
                break ;
            end
            sm_image_buf{k}.trend(eno, :, :) = cur_sm_arr(:, :);
            % compute score for each trend
            for sno=1:sm_num
                tmp_sc = scoreEstimation(sm_image_buf{k}.trend(eno, :, sno));
                if isnan(tmp_sc)
                    sm_image_buf{k}.trend(eno, :, sno)
                else
                    sm_image_buf{k}.score(eno, sno) = tmp_sc;
                end
            end
            % normalize
            %                     score_min = min(sm_image_buf{k}.score(eno, :));
            %                     score_max = max(sm_image_buf{k}.score(eno, :));
            %                     sm_image_buf{k}.score(eno, :) = (sm_image_buf{k}.score(eno, :) - score_min) / (score_max - score_min);
            %                     sm_image_buf{k}.score(eno, :) = sm_image_buf{k}.score(eno, :) / sum(sm_image_buf{k}.score(eno, :));
        end
        %
        if k_stop == false
            % visualize
            %             close all;
            %             figure;
            %             imagesc(sm_image_buf{k}.image.image);
            %             hold on
            %             scatter(sm_image_buf{k}.points(:, 1), ...
            %                 sm_image_buf{k}.points(:, 2), ...
            %                 3*ones(1,size(sm_image_buf{k}.points, 1)), sm_image_buf{k}.points(:, 3), 'filled');
            %             %                     pause(1)
            %             for eno = 1:err_num
            %                 figure;
            %                 tmp_buf(:,:) = sm_image_buf{k}.trend(eno, :, :);
            %                 min_val = min(tmp_buf, [], 1);
            %                 subplot(1,2,1)
            %                 plot(-step_num : step_num, tmp_buf(:, 1:4) - repmat(min_val(1:4), size(sm_image_buf{k}.trend, 2), 1), '-o');
            %                 hold on;
            %                 plot(-step_num : step_num, tmp_buf(:, 5:10) - repmat(min_val(5:10), size(sm_image_buf{k}.trend, 2), 1), '-x');
            %                 plot(-step_num : step_num, tmp_buf(:, 11:13) - repmat(min_val(11:13), size(sm_image_buf{k}.trend, 2), 1), '-.');
            %                 %                         ylim([sm_min sm_max]);
            %                 legend(para.sm_list);
            %                 subplot(1,2,2)
            %                 plot(sm_image_buf{k}.score(eno,:))
            %             end
            %                             figure;
            %                             plot(sm_image_buf{1, 1}.score(1,:))
            k = k + 1;
        else
            sm_image_buf{k} = [];
        end
    end
end


%% clustering w\ trend
dist_mat = zeros(length(sm_image_buf), length(sm_image_buf));
for i=1:length(sm_image_buf)
    for j=i+1:length(sm_image_buf)
        dist = ones(1, length(sm_list)) * 9999;
        weight = zeros(1, length(sm_list));
        % compute distance for i and j
        %         for eno = 1:err_num
        %             if sum(sum(sm_image_buf{i}.trend(eno, :, :))) > 0 && ...
        %                     sum(sum(sm_image_buf{j}.trend(eno, :, :))) > 0
        %                 %                 V = max(sm_image_buf{i}.score(eno, :), dist_thres) - max(sm_image_buf{j}.score(eno, :), dist_thres);
        %                 V = sm_image_buf{i}.score(eno, :) - sm_image_buf{j}.score(eno, :);
        %                 dist(eno) = sqrt(V * V');
        %             end
        %         end
        %         dist_mat(i, j) = sum(dist);
        for sno = 1:length(sm_list)
            if sum(sum(sm_image_buf{i}.trend(:, sno, :))) > 0 && ...
                    sum(sum(sm_image_buf{j}.trend(:, sno, :))) > 0
                %                 V = max(sm_image_buf{i}.score(eno, :), dist_thres) - max(sm_image_buf{j}.score(eno, :), dist_thres);
                tmp_score_i = mean(sm_image_buf{i}.score(:, sno));
                %                 [~, tmp_idx_i] = sort(tmp_score_i);
                tmp_score_j = mean(sm_image_buf{j}.score(:, sno));
                %                 [~, tmp_idx_j] = sort(tmp_score_i);
                %
                tmp_weight = exp((tmp_score_i + tmp_score_j) / 2);
                weight(sno) = tmp_weight;
                dist(sno) = abs(tmp_score_i - tmp_score_j);
            end
        end
        weight = weight / sum(weight);
        dist_mat(i, j) = weight * dist';%sum(dist);
    end
end
dist_mat = triu(dist_mat, -1) + triu(dist_mat)';

% for eno = 1:err_num
%     figure;
%     hold on;
%     for i=1:length(sm_image_buf)
%         scatter3(sm_image_buf{i}.score(eno, 1), sm_image_buf{i}.score(eno, 5), i);
%     end
% end

Z = linkage(dist_mat, 'centroid');
figure;
dendrogram(Z);
T = cluster(Z,'maxclust',24);
figure;
histogram(T, length(unique(T)))

% set up cluster arrays
cidx = unique(T);
for i=1:length(cidx)
    %     cluster{i}.image = sm_image_buf{cidx(i)}.image;
    %     cluster{i}.points = sm_image_buf{cidx(i)}.points;
    cluster{i}.count = 0;
    cluster{i}.patch_idx = [];
    cluster{i}.trend_avg(:, :, :) = zeros(err_num, 2*step_num+1, sm_num);
    %     figure;
    %     imagesc(repmat(sm_image_buf{i}.image,[1,1,3]));
end

dir_output = ['C:\Users\yzhao347\Copy\cluster_kitti\'];
save([dir_output 'data.mat'], 'sm_image_buf', '-v7.3');

for i=1:length(sm_image_buf)
    i
    %     if T(i) ~= 22 && T(i) ~= 23 && T(i) ~= 27
    %         continue;
    %     end
    %     if T(i) ~= 9
    %         continue ;
    %     end
    % 1. visualize the center of each cluster
    close all;
    figure;
    subplot(1,2,1)
    imagesc(sm_image_buf{i}.image.image);
    hold on
    scatter((sm_image_buf{i}.points(:, 1)), ...
        (sm_image_buf{i}.points(:, 2)), ...
        3*ones(1,size(sm_image_buf{i}.points, 1)), (sm_image_buf{i}.points(:, 3)), 'filled');
    %
    %     patch_name = sprintf('frameno %04d, score [', sm_image_buf{i}.image.frame_no);
    %     for sno=1:length(sm_list)
    %     score_str = sprintf('%.02f ', mean(sm_image_buf{i}.score(:, sno)));
    %     patch_name = [patch_name, score_str];
    %     end
    %     patch_name = [patch_name, ']'];
    %     title(patch_name);
    title(['frameno ' num2str(sm_image_buf{i}.image.frame_no)]);
    subplot(1,2,2)
    plot(mean(sm_image_buf{i}.score(:, :), 1), '--o');
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 28 9])
    %
    dir_cluster = [dir_output 'cluster_' num2str(T(i)) '\'];
    %     if ~exist(dir_cluster)
    mkdir([dir_cluster]);
    %     end
    file_name = sprintf('Patch_%d.jpg', i);
    saveas(gcf, [dir_cluster file_name]);
    %
    for eno = 1:err_num
        figure;
        cur_sm_arr(:, :) = sm_image_buf{i}.trend(eno, :, :);
        min_val = min(cur_sm_arr(:, :), [], 1);
        plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
        hold on;
        plot(-step_num : step_num, cur_sm_arr(:, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1), 1), '-x');
        plot(-step_num : step_num, cur_sm_arr(:, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1), 1), '-.');
        ylim([sm_min sm_max]);
        legend(para.sm_list);
        %
        err_frame = ['error_' para.err_list{eno} '\'];
        mkdir([dir_cluster, err_frame]);
        file_name = sprintf('Trend_%s_%d', para.err_list{eno}, i);
        saveas(gcf, [dir_cluster err_frame file_name]);
        %
        cluster{T(i)}.trend_avg(eno, :, :) = ...
            cluster{T(i)}.trend_avg(eno, :, :) + sm_image_buf{i}.trend(eno, :, :);
    end
    %
    score_name = sprintf('Score_%d.mat', i);
    score = sm_image_buf{i}.score(:, :);
    save([dir_cluster score_name], 'score');
    %
    cluster{T(i)}.patch_idx = [cluster{T(i)}.patch_idx, i];
    cluster{T(i)}.count = cluster{T(i)}.count + 1;
    %
    %     fid = fopen(sm_image_buf{i}.image.fname, 'w');
    %     fwrite(fid, T(i), 'uint8');
    %     fclose (fid);
end

save([dir_output 'cluster.mat'], 'cluster', '-v7.3');

close all;
for i=1:length(cluster)
    figure;
    cur_sm_arr(:, :) = sum(cluster{i}.trend_avg(:, :, :), 1) / cluster{i}.count;
    min_val = min(cur_sm_arr(:, :), [], 1);
    plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
    hold on;
    plot(-step_num : step_num, cur_sm_arr(:, 5:10) - repmat(min_val(5:10), size(cur_sm_arr, 1), 1), '-x');
    plot(-step_num : step_num, cur_sm_arr(:, 11:13) - repmat(min_val(11:13), size(cur_sm_arr, 1), 1), '-.');
    %     ylim([sm_min sm_max]);
    legend(para.sm_list);
    title(['average trend of cluster ' num2str(i) ' patch number ' num2str(cluster{i}.count)]);
    %     dir_cluster = [dir_output 'cluster_' num2str(i) '\'];
    file_name = sprintf('Trend_cluster_%d.jpg', i);
    saveas(gcf, [dir_output file_name]);
end
