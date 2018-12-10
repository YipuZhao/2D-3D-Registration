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
    'site1_v';
    'site2_v';
    'site3_v';
    'site4_v';
    'run3_v';
    'Pandey';
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
%     'Inten-Ref'; 'H-Ref'; 'S-Ref'; 'V-Ref';
%     'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
%     'H-Nx'; 'H-Ny'; 'H-Nz';
%     'GradD-Curv'; 'GradM-Curv';
%     };
sm_list = {...
    'Inten-Ref'; 'IntenEQ-Ref'; 'Y-Ref'; 'H-Ref';
    'Inten-Nx'; 'Inten-Ny'; 'Inten-Nz';
    %     'IntenEQ-Nx'; 'IntenEQ-Ny'; 'IntenEQ-Nz';
    %     'H-Nx'; 'H-Ny'; 'H-Nz';
    'HGradD-CurvEQ'; 'HGradM-CurvEQ';
    %     'GradD-Curv'; 'GradM-Curv';
    %     'GradDFil-Curv'; 'GradMFil-Curv';
    %     'Inten-RefEQ'; 'IntenEQ-RefEQ';
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


%% load trend data for each frame
k = 1;
for di=2:5
    % di = 5;
    dir_trend = ['..\..\data\GT_reorg\similarity\' data_list{di} '_output\'];
    % dir_trend = ['.\data\' data_list{di} '_output\'];
    
    para = loadPara(data_list{di}, sm_list, err_list);
    [points, img_list] = loadData(para.dir_name, para.dir_image_name, para);
    
    grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
    grid_x(end) = para.ROI_width(2)+1;
    grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
    grid_y(end) = para.ROI_height(2)+1;
    
    frame_num = length(img_list);
    sm_num = length(para.sm_list);
    err_num = length(para.err_list);
    
    % sm_grid_buf = zeros(frame_num, 2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
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
        [u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
        
        inten_img = rgb2gray(image.image);
        inten_img_eq = adapthisteq(inten_img);
        % image - illumination inv
        tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
        y_img = tmp_img(:,:,1);
        tmp_img = rgb2hsv(double(image.image));
        h_img = tmp_img(:,:,1);
        % image - gradient
        [gm_img_hue, gd_img_hue] = imgradient(h_img);
        
        %         ref_arr = points(points_id(:), 6);
        x_pcd_arr = double(points(points_id,2));
        y_pcd_arr = double(points(points_id,3));
        z_pcd_arr = double(points(points_id,4));
        ref_pcd_arr = double(points(points_id,5)); % refc
        ref_pcd_arr_eq = double(points(points_id,6));
        % point cloud - normal
        nx_pcd_arr = double(points(points_id,7));
        ny_pcd_arr = double(points(points_id,8));
        nz_pcd_arr = double(points(points_id,9));
        % convert normal from wc to vc
        norm_VC = inv_R_w2v * [nx_pcd_arr, ny_pcd_arr, nz_pcd_arr]';
        nx_pcd_arr = norm_VC(1, :)';
        ny_pcd_arr = norm_VC(2, :)';
        nz_pcd_arr = norm_VC(3, :)';
        % point cloud - curvature
        curv_pcd_arr = double(points(points_id,10)); % cur
        curv_pcd_arr_eq = histeq(curv_pcd_arr);
        % point cloud - edge
        edge_pcd_arr = double(points(points_id,11));
            max_curv = max(curv_pcd_arr_eq);
            curv_pcd_arr_eq = curv_pcd_arr_eq + edge_pcd_arr * max_curv;
            
        dir_frame = ['frame_' num2str(frame_no) '\'];
        for i = 1:para.grid_row_num
            for j = 1:para.grid_col_num
                k_stop = false;
                %
                xbias = -para.ROI_width(1)+1;
                ybias = -para.ROI_height(1)+1;
                % points for grid[i][j]
                grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
                % remove 2D patches without enough 3D points projected
                if length(ref_pcd_arr_eq(grid_points_idx)) < points_thres
                    continue ;
                end
                sm_grid_buf{k}.points(:, :) = [
                    u(grid_points_idx)'+xbias-grid_x(j), ...
                    v(grid_points_idx)'+ybias-grid_y(i), ...
                    x_pcd_arr(grid_points_idx), ...
                    y_pcd_arr(grid_points_idx), ...
                    z_pcd_arr(grid_points_idx), ...
                    ref_pcd_arr(grid_points_idx), ...
                    nx_pcd_arr(grid_points_idx),...
                    ny_pcd_arr(grid_points_idx),...
                    nz_pcd_arr(grid_points_idx),...
                    curv_pcd_arr_eq(grid_points_idx)
                    ];
                %
                sm_grid_buf{k}.image.image = inten_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.frame_no = image.frame_no;
                sm_grid_buf{k}.image.UTC = image.UTC;
                sm_grid_buf{k}.image.xyz = image.xyz;
                sm_grid_buf{k}.image.prh = image.prh;
                %
                sm_grid_buf{k}.image.img_attribute{1} = inten_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.img_attribute{2} = inten_img_eq(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.img_attribute{3} = y_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.img_attribute{4} = h_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.img_attribute{5} = gd_img_hue(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                sm_grid_buf{k}.image.img_attribute{6} = gm_img_hue(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                    grid_x(j)+xbias:grid_x(j+1)+xbias-1);
                %
                for eno = 1:err_num
                    err_frame = ['error_' para.err_list{eno} '\'];
                    %
                    file_name = sprintf('SM_frame_%04d_%s', frame_no, para.err_list{eno});
                    load([dir_trend, dir_frame, err_frame, file_name '.mat']);
                    %
                    if min(min(cur_sm_grid(:, :, i, j))) == 0
                        k_stop = true;
                        break ;
                    end
                    sm_grid_buf{k}.trend(eno, :, :) = cur_sm_grid(:, :, i, j);
                    % compute score for each trend
                    for sno=1:sm_num
                        sm_grid_buf{k}.score(eno, sno) = scoreEstimation(sm_grid_buf{k}.trend(eno, :, sno));
                    end
                    % normalize
                    %                     score_min = min(sm_grid_buf{k}.score(eno, :));
                    %                     score_max = max(sm_grid_buf{k}.score(eno, :));
                    %                     sm_grid_buf{k}.score(eno, :) = (sm_grid_buf{k}.score(eno, :) - score_min) / (score_max - score_min);
                    %                     sm_grid_buf{k}.score(eno, :) = sm_grid_buf{k}.score(eno, :) / sum(sm_grid_buf{k}.score(eno, :));
                end
                %
                if k_stop == false
                    % visualize
                    %                 close all;
                    %                 figure;
                    %                 imagesc(repmat(sm_grid_buf{k}.image,[1,1,3]));
                    %                 hold on
                    %                 scatter(sm_grid_buf{k}.points(:, 1), ...
                    %                     sm_grid_buf{k}.points(:, 2), ...
                    %                     3*ones(1,size(sm_grid_buf{k}.points, 1)), sm_grid_buf{k}.points(:, 3), 'filled');
                    %                 %                     pause(1)
                    %                 for eno = 1:err_num
                    %                     figure;
                    %                     tmp_buf(:,:) = sm_grid_buf{k}.trend(eno, :, :);
                    %                     min_val = min(tmp_buf, [], 1);
                    %                     subplot(1,2,1)
                    %                     plot(-step_num : step_num, tmp_buf(:, 1:4) - repmat(min_val(1:4), size(sm_grid_buf{k}.trend, 2), 1), '-o');
                    %                     hold on;
                    %                     plot(-step_num : step_num, tmp_buf(:, 5:7) - repmat(min_val(5:7), size(sm_grid_buf{k}.trend, 2), 1), '-x');
                    %                     plot(-step_num : step_num, tmp_buf(:, 8:9) - repmat(min_val(8:9), size(sm_grid_buf{k}.trend, 2), 1), '-.');
                    %                     %                         ylim([sm_min sm_max]);
                    %                     legend(para.sm_list);
                    %                     subplot(1,2,2)
                    %                     plot(sm_grid_buf{k}.score(eno,:))
                    %                 end
                    %                 figure;
                    %                 plot(sm_grid_buf{1, 1}.score(1,:))
                    k = k + 1;
                else
                    sm_grid_buf{k} = [];
                end
            end
        end
    end
end


%% clustering w\ trend
dist_mat = zeros(length(sm_grid_buf), length(sm_grid_buf));
for i=1:length(sm_grid_buf)
    for j=i+1:length(sm_grid_buf)
        dist = ones(1, err_num) * 9999;
        % compute distance for i and j
        %         for eno = 1:err_num
        %             %             for sno=1:sm_num
        %             %                 if sum(sm_grid_buf{i}.trend(eno, :, sno)) > 0 && ...
        %             %                         sum(sm_grid_buf{j}.trend(eno, :, sno)) > 0
        %             %                     % get the derivative of trend_i_eno_sno
        %             %                     %                     der_i = diff(sm_grid_buf{i}.trend(eno, :, sno));
        %             %                     % get the derivative of trend_j_eno_sno
        %             %                     %                     der_j = diff(sm_grid_buf{j}.trend(eno, :, sno));
        %             %                     % compute dist(trend_i_eno_sno, trend_j_eno_sno)
        %             %                     %                     dist(eno,sno) = w * abs(der_i - der_j)';
        %             %
        %             %
        %             %                 end
        %             %             end
        %             if sum(sum(sm_grid_buf{i}.trend(eno, :, :))) > 0 && ...
        %                     sum(sum(sm_grid_buf{j}.trend(eno, :, :))) > 0
        %                 V = sm_grid_buf{i}.score(eno, :) - sm_grid_buf{j}.score(eno, :);
        %                 dist(eno) = sqrt(V * V');
        %             end
        %         end
        %         dist_mat(i, j) = sum(dist);
        for eno = 1:err_num
            if sum(sum(sm_grid_buf{i}.trend(eno, :, :))) > 0 && ...
                    sum(sum(sm_grid_buf{j}.trend(eno, :, :))) > 0
                %                 V = max(sm_grid_buf{i}.score(eno, :), dist_thres) - max(sm_grid_buf{j}.score(eno, :), dist_thres);
                V = sm_grid_buf{i}.score(eno, :) - sm_grid_buf{j}.score(eno, :);
                dist(eno) = sqrt(V * V');
            end
        end
        dist_mat(i, j) = sum(dist);
    end
end
dist_mat = triu(dist_mat, -1) + triu(dist_mat)';

% for eno = 1:err_num
%     figure;
%     hold on;
%     for i=1:length(sm_grid_buf)
%         scatter3(sm_grid_buf{i}.score(eno, 1), sm_grid_buf{i}.score(eno, 5), i);
%     end
% end

Z = linkage(dist_mat, 'complete');
figure;
dendrogram(Z);
T = cluster(Z,'maxclust',10);
figure;
histogram(T)

% set up cluster arrays
cidx = unique(T);
for i=1:length(cidx)
    %     cluster{i}.image = sm_grid_buf{cidx(i)}.image;
    %     cluster{i}.points = sm_grid_buf{cidx(i)}.points;
    cluster{i}.count = 0;
    cluster{i}.patch_idx = [];
    cluster{i}.trend_avg(:, :, :) = zeros(err_num, 2*step_num+1, sm_num);
    %     figure;
    %     imagesc(repmat(sm_grid_buf{i}.image,[1,1,3]));
end

dir_output = ['C:\Users\yzhao347\Copy\cluster_atl_v\'];
save([dir_output 'data.mat'], 'sm_grid_buf');
for i=1:length(sm_grid_buf)
    i
    %     if T(i) ~= 9
    %         continue ;
    %     end
    % 1. visualize the center of each cluster
    close all;
    figure;
    imagesc(repmat(sm_grid_buf{i}.image,[1,1,3]));
    hold on
    scatter((sm_grid_buf{i}.points(:, 1)), ...
        (sm_grid_buf{i}.points(:, 2)), ...
        3*ones(1,size(sm_grid_buf{i}.points, 1)), (sm_grid_buf{i}.points(:, 3)), 'filled');
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
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
        cur_sm_arr(:, :) = sm_grid_buf{i}.trend(eno, :, :);
        min_val = min(cur_sm_arr(:, :), [], 1);
        plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
        hold on;
        plot(-step_num : step_num, cur_sm_arr(:, 5:7) - repmat(min_val(5:7), size(cur_sm_arr, 1), 1), '-x');
        plot(-step_num : step_num, cur_sm_arr(:, 8:9) - repmat(min_val(8:9), size(cur_sm_arr, 1), 1), '-.');
        ylim([sm_min sm_max]);
        legend(para.sm_list);
        %
        err_frame = ['error_' para.err_list{eno} '\'];
        mkdir([dir_cluster, err_frame]);
        file_name = sprintf('Trend_%s_%d', para.err_list{eno}, i);
        saveas(gcf, [dir_cluster err_frame file_name]);
        %
        cluster{T(i)}.trend_avg(eno, :, :) = ...
            cluster{T(i)}.trend_avg(eno, :, :) + sm_grid_buf{i}.trend(eno, :, :);
    end
    %
    score_name = sprintf('Score_%d.mat', i);
    score = sm_grid_buf{i}.score(:, :);
    save([dir_cluster score_name], 'score');
    
    cluster{T(i)}.patch_idx = [cluster{T(i)}.patch_idx, i];
    cluster{T(i)}.count = cluster{T(i)}.count + 1;
end

save([dir_output 'cluster.mat'], 'cluster');

close all;
for i=1:length(cluster)
    figure;
    cur_sm_arr(:, :) = sum(cluster{i}.trend_avg(:, :, :), 1) / cluster{i}.count;
    min_val = min(cur_sm_arr(:, :), [], 1);
    plot(-step_num : step_num, cur_sm_arr(:, 1:4) - repmat(min_val(1:4), size(cur_sm_arr, 1), 1), '-o');
    hold on;
    plot(-step_num : step_num, cur_sm_arr(:, 5:7) - repmat(min_val(5:7), size(cur_sm_arr, 1), 1), '-x');
    plot(-step_num : step_num, cur_sm_arr(:, 8:9) - repmat(min_val(8:9), size(cur_sm_arr, 1), 1), '-.');
    ylim([sm_min sm_max]);
    legend(para.sm_list);
    title(['average trend of cluster ' num2str(i) ' patch number ' num2str(cluster{i}.count)]);
    %     dir_cluster = [dir_output 'cluster_' num2str(i) '\'];
    file_name = sprintf('Trend_cluster_%d.jpg', i);
    saveas(gcf, [dir_output file_name]);
end
