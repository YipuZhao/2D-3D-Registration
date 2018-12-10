clear all;
close all;

%% pre-defined parameters
sm_min = 0.8;
sm_max = 1.4;
step_num = 20;
adj_step = 0;%4;%2;
points_thres = 1000;

%
alpha = 0.001;
score_thres = -0.1;%0.04;
dist_thres = -0.1;%0.02;
% upbound_side_score = 0.2;
% alpha = 5;
base_w = 50
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

% err_list = {...
%     'heading';
%     'rolling';
%     'pitching';
%     'shift_x';
%     'shift_y';
%     'shift_z';
%     };
err_list = {...
    'heading';
    };


%% load trend data for each frame
k = 1;
% for di=2:5
di = 5;
dir_trend = ['D:\Copy\' data_list{di} '_output\'];
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
    tmp_img = rgb2gray(image.image);
    inten_img = tmp_img(para.ROI_height(1):para.ROI_height(2), para.ROI_width(1):para.ROI_width(2));
    R = makeRotationMatrix(para.x0(4:6));
    inv_R = inv(R);
    t = para.x0(1:3)';
    [u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
    ref_arr = points(points_id(:), 6);
    %         figure;
    %         imagesc(image.image);
    %         hold on
    %         scatter(u, v, ...
    %             3*ones(1,length(u)), ref_arr, 'filled');
    %
    dir_frame = ['frame_' num2str(frame_no) '\'];
    for i = 1:para.grid_row_num
        for j = 1:para.grid_col_num
            k_stop = false;
            %
            xbias = -para.ROI_width(1)+1;
            ybias = -para.ROI_height(1)+1;
            % points for grid[i][j]
            grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
            %                 ref_pcd_arr = log(points(points_id(grid_points_idx), 5));
            %                 ref_pcd_arr(ref_pcd_arr==0) = 0;
            %                 ref_pcd_arr = points(points_id(grid_points_idx), 6);
            % remove 2D patches without enough 3D points projected
            if length(ref_arr(grid_points_idx)) < points_thres
                continue ;
            end
            sm_grid_buf{k}.points(:, :) = [u(grid_points_idx)'+xbias-grid_x(j), ...
                v(grid_points_idx)'+ybias-grid_y(i), ...
                ref_arr(grid_points_idx)];
            %
            sm_grid_buf{k}.image = inten_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
                grid_x(j)+xbias:grid_x(j+1)+xbias-1);
            %
            for eno = 1:err_num
                err_frame = ['error_' para.err_list{eno} '\'];
                %
                file_name = sprintf('SM_frame_%4d_%s', frame_no, para.err_list{eno});
                load([dir_trend, dir_frame, err_frame, file_name '.mat']);
                %
                if min(min(cur_sm_grid(:, :, i, j))) == 0
                    k_stop = true;
                    break ;
                end
                sm_grid_buf{k}.trend(eno, :, :) = cur_sm_grid(:, :, i, j);
                % compute score for each trend
                score_tmp = zeros(1+2*adj_step, sm_num);
                for ano = -adj_step : adj_step
                    for sno=1:sm_num
                        %                             sm_der = diff(sm_grid_buf{k}.trend(eno, :, sno));
                        sm_der = gradient(sm_grid_buf{k}.trend(eno, :, sno));
                        w = [base_w - (step_num + ano) : 1 : base_w, ...
                            base_w-1 : -1 : base_w - (step_num - ano)];
                        w = w / sum(w) * 100;
                        s1 = sm_der(1:step_num+ano);
                        s2 = sm_der(step_num+1+ano:end);
                        %                             score_tmp(ano + adj_step + 1, sno) = w * [s1, -s2]' ...
                        %                                 - alpha * ( std(s1)/abs(mean(s1)) + std(s2)/abs(mean(s2)) );
                        t1 = w(1:length(s1)) * s1';
                        t2 = w(1:length(s2))* [-s2]';
                        score_tmp(ano + adj_step + 1, sno) = t1 + t2 ...
                            - alpha * ( std(s1)/abs(mean(s1)) + std(s2)/abs(mean(s2)) ) ...
                            - abs(sum(sm_der));
                        
                    end
                end
                score_adj = max(score_tmp, score_thres);
                %                     %                     [msc, midx] = max(sum(score_tmp(val_idx), 2));
                %                     [msc, midx] = max(sum(val_idx, 2));
                %                     if msc > score_thres
                %                         sm_grid_buf{k}.score(eno, :) = score_tmp(midx, :);
                %                     else
                %                         sm_grid_buf{k}.score(eno, :) = score_tmp(ano+1, :);
                %                     end
                [msc, midx] = max(sum(score_adj, 2));
                sm_grid_buf{k}.score(eno, :) = score_tmp(midx, :);
                % normalize
                %                     score_min = min(sm_grid_buf{k}.score(eno, :));
                %                     score_max = max(sm_grid_buf{k}.score(eno, :));
                %                     sm_grid_buf{k}.score(eno, :) = (sm_grid_buf{k}.score(eno, :) - score_min) / (score_max - score_min);
                %                     sm_grid_buf{k}.score(eno, :) = sm_grid_buf{k}.score(eno, :) / sum(sm_grid_buf{k}.score(eno, :));
            end
            %
            if k_stop == false
                % visualize
                close all;
                figure;
                imagesc(repmat(sm_grid_buf{k}.image,[1,1,3]));
                hold on
                scatter(sm_grid_buf{k}.points(:, 1), ...
                    sm_grid_buf{k}.points(:, 2), ...
                    3*ones(1,size(sm_grid_buf{k}.points, 1)), sm_grid_buf{k}.points(:, 3), 'filled');
                %                     pause(1)
                for eno = 1:err_num
                    figure;
                    tmp_buf(:,:) = sm_grid_buf{k}.trend(eno, :, :);
                    min_val = min(tmp_buf, [], 1);
                    subplot(1,2,1)
                    plot(-step_num : step_num, tmp_buf(:, 1:4) - repmat(min_val(1:4), size(sm_grid_buf{k}.trend, 2), 1), '-o');
                    hold on;
                    plot(-step_num : step_num, tmp_buf(:, 5:7) - repmat(min_val(5:7), size(sm_grid_buf{k}.trend, 2), 1), '-x');
                    plot(-step_num : step_num, tmp_buf(:, 8:9) - repmat(min_val(8:9), size(sm_grid_buf{k}.trend, 2), 1), '-.');
                    %                         ylim([sm_min sm_max]);
                    legend(para.sm_list);
                    subplot(1,2,2)
                    plot(sm_grid_buf{k}.score(eno,:))
                end
%                 figure;
%                 plot(sm_grid_buf{1, 1}.score(1,:))
                k = k + 1;
            else
                sm_grid_buf{k} = [];
            end
        end
    end
end
% end

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
                V = max(sm_grid_buf{i}.score(eno, :), dist_thres) - max(sm_grid_buf{j}.score(eno, :), dist_thres);
                dist(eno) = sqrt(V * V');
            end
        end
        dist_mat(i, j) = sum(dist);
    end
end
dist_mat = triu(dist_mat, -1) + triu(dist_mat)';

for eno = 1:err_num
    figure;
    hold on;
    for i=1:length(sm_grid_buf)
        scatter3(sm_grid_buf{i}.score(eno, 1), sm_grid_buf{i}.score(eno, 11), i);
    end
end

Z = linkage(dist_mat, 'complete');
figure;
dendrogram(Z);
T = cluster(Z,'maxclust',28);
figure;
histogram(T)

% set up cluster arrays
cidx = unique(T);
for i=1:length(cidx)
    %     cluster{i}.image = sm_grid_buf{cidx(i)}.image;
    %     cluster{i}.points = sm_grid_buf{cidx(i)}.points;
    cluster{i}.count = 0;
    cluster{i}.trend_avg(:, :, :) = zeros(err_num, 2*step_num+1, sm_num);
    %     figure;
    %     imagesc(repmat(sm_grid_buf{i}.image,[1,1,3]));
end

dir_output = [dir_trend '\clusters\'];
for i=1:length(sm_grid_buf)
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
        clear plot_buf
        plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 1:4);
        plot(-step_num : step_num, plot_buf, '-o');
        hold on;
        clear plot_buf
        plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 5:7);
        plot(-step_num : step_num, plot_buf, '-x');
        clear plot_buf
        plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 8:9);
        plot(-step_num : step_num, plot_buf, '-.');
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
    cluster{T(i)}.count = cluster{T(i)}.count + 1;
end

save([dir_output 'cluster.mat'], 'cluster');

close all;
for i=1:length(cluster)
    figure;
    clear plot_buf
    plot_buf(:, :) = cluster{i}.trend_avg(eno, :, 1:4) / cluster{i}.count;
    plot(-step_num : step_num, plot_buf, '-o');
    hold on;
    clear plot_buf
    plot_buf(:, :) = cluster{i}.trend_avg(eno, :, 5:7) / cluster{i}.count;
    plot(-step_num : step_num, plot_buf, '-x');
    clear plot_buf
    plot_buf(:, :) = cluster{i}.trend_avg(eno, :, 8:9) / cluster{i}.count;
    plot(-step_num : step_num, plot_buf, '-.');
    ylim([sm_min sm_max]);
    legend(para.sm_list);
    title(['average trend of cluster ' num2str(i) ' patch number ' num2str(cluster{i}.count)]);
    %     dir_cluster = [dir_output 'cluster_' num2str(i) '\'];
    file_name = sprintf('Trend_cluster_%d.jpg', i);
    saveas(gcf, [dir_output file_name]);
end

% for k=3:6
% find the proper cluster number k
% k = 12;
% [inds, cidx] = kmedioids(dist_mat, k);
% histogram(inds);
% % set up cluster arrays
% for i=1:length(cidx)
%     cluster{i}.image = sm_grid_buf{cidx(i)}.image;
%     cluster{i}.points = sm_grid_buf{cidx(i)}.points;
%     cluster{i}.count = 0;
%     cluster{i}.trend_avg(:, :, :) = zeros(err_num, 2*step_num, sm_num);
%     figure;
%     imagesc(repmat(sm_grid_buf{i}.image,[1,1,3]));
% end
% for i=1:length(sm_grid_buf)
%     % 1. visualize the center of each cluster
%     close all;
%     figure;
%     imagesc(repmat(sm_grid_buf{i}.image,[1,1,3]));
%     hold on
%     scatter(fliplr(sm_grid_buf{i}.points(:, 1)), ...
%         fliplr(sm_grid_buf{i}.points(:, 2)), ...
%         3*ones(1,size(sm_grid_buf{i}.points, 1)), flipud(sm_grid_buf{i}.points(:, 3)), 'filled');
%     set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
%     %
%     dir_cluster = ['cluster_' num2str(inds(i)) '\'];
%     %     if ~exist(dir_cluster)
%     mkdir([para.dir_output, dir_cluster]);
%     %     end
%     file_name = sprintf('Patch_%d.jpg', i);
%     saveas(gcf, [para.dir_output dir_cluster file_name]);
%     %
%     for eno = 1:err_num
%         figure;
%         clear plot_buf
%         plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 1:4);
%         plot(-step_num : step_num, plot_buf, '-o');
%         hold on;
%         clear plot_buf
%         plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 5:10);
%         plot(-step_num : step_num, plot_buf, '-x');
%         clear plot_buf
%         plot_buf(:, :) = sm_grid_buf{i}.trend(eno, :, 11:12);
%         plot(-step_num : step_num, plot_buf, '-.');
%         ylim([sm_min sm_max]);
%         legend(para.sm_list);
%         %
%         err_frame = ['error_' para.err_list{eno} '\'];
%         mkdir([para.dir_output, dir_cluster, err_frame]);
%         file_name = sprintf('Trend_%s_%d', para.err_list{eno}, i);
%         saveas(gcf, [para.dir_output dir_cluster err_frame file_name]);
%     end
%     %
%     cluster{inds(i)}.trend_avg(eno, :, :) = ...
%         cluster{inds(i)}.trend_avg(eno, :, :) + sm_grid_buf{i}.trend(eno, :, :);
% end

% pull out the data under each cluster
%     for i=1:k
%         tmp_center = sm_grid_buf{cidx(i)}
%         % 1. visualize the center of each cluster
%         close all;
%         figure;
%         imagesc(repmat(tmp_center.image,[1,1,3]));
%         hold on
%         scatter(fliplr(tmp_center.points(:, 1)), ...
%             fliplr(tmp_center.points(:, 2)), ...
%             3*ones(1,size(tmp_center.points, 1)), flipud(tmp_center.points(:, 3)), 'filled');
%         pause(1)
%         for eno = 1:err_num
%             figure;
%             clear plot_buf
%             plot_buf(:, :) = tmp_center.trend(eno, :, 1:4);
%             plot(-step_num : step_num, plot_buf, '-o');
%             hold on;
%             clear plot_buf
%             plot_buf(:, :) = tmp_center.trend(eno, :, 5:10);
%             plot(-step_num : step_num, plot_buf, '-x');
%             clear plot_buf
%             plot_buf(:, :) = tmp_center.trend(eno, :, 11:12);
%             plot(-step_num : step_num, plot_buf, '-.');
%             ylim([sm_min sm_max]);
%             legend(para.sm_list);
%         end
%         % 2. display the number of points in each cluster
%         ptnum = sum(inds == i)
%     end
% end
