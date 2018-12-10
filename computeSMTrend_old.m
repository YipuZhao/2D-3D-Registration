function [sm_grid] = computeSMTrend_old(img_list, points, para)

show_img = true

% t, inv_R: lidar to cam
% R = makeRotationMatrix(para.x0(4:6));
% inv_R = inv(R);
% t = para.x0(1:3)';

sm_num = length(para.sm_list);
frame_num = length(img_list);
nbins = round(256/para.bin_fraction);
grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
grid_x(end) = para.ROI_width(2)+1;
grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
grid_y(end) = para.ROI_height(2)+1;

step_num = 0;
d_heading = 0.1;
tmp_x0 = para.x0;

% (img_idx, iter_no, sn, i, j)
sm_grid = zeros(frame_num, 2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
sm_arr = zeros(frame_num, 2*step_num+1, sm_num);

for img_idx = 2:frame_num
    image = img_list{img_idx};
    frame_no = image.frame_no
    %
    dir_frame = ['frame_' num2str(frame_no) '\'];
    mkdir([para.dir_output dir_frame]);
    %     iter_no = 1;
    for iter_no = 1:1+2*step_num
        err_heading = d_heading * (iter_no-step_num-1)
        %% add error to registration parameters
        tmp_pitch = para.x0(4);
        tmp_roll = para.x0(5);
        tmp_heading = para.x0(6) + deg2rad(err_heading);
        %         tmp_x0(4) = para.x0(4) + deg2rad(err_heading)
        R = makeRotationMatrix([tmp_pitch, tmp_roll, tmp_heading]);
        inv_R = inv(R);
        t = tmp_x0(1:3)';
        
        %         tic
        [u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
        %         toc
        h = fspecial('gaussian',25*[1,1],6);
        
        %         tic
        % image - intensity
        inten_img = rgb2gray(image.image);
        % image - illumination inv
        tmp_img = rgb2ill(double(image.image));
        illu_img{1} = tmp_img(:,:,1);
        illu_img{2} = tmp_img(:,:,2);
        illu_img{3} = tmp_img(:,:,3);
        % image - gradient
        [gm_img, ~] = imgradient(inten_img);
        gm_img_upd = log(double(gm_img)); % gradient magnitude
        gm_img_upd(gm_img==0) = 0;
        gm_img_upd = imfilter(gm_img_upd,h,'replicate');
        [gx_img, gy_img] = imgradientxy(inten_img);
        gx_img = abs(gx_img);
        gx_img_upd = log(double(gx_img)); % gradient magnitude
        gx_img_upd(gx_img==0) = 0;
        gx_img_upd = imfilter(gx_img_upd,h,'replicate');
        gy_img = abs(gy_img);
        gy_img_upd = log(double(gy_img)); % gradient magnitude
        gy_img_upd(gy_img==0) = 0;
        gy_img_upd = imfilter(gy_img_upd,h,'replicate');
        %         % smooth
        %         h = fspecial('gaussian',25*[1,1],6);
        %         gm_img_upd = imfilter(gm_img_upd,h,'replicate');
        %         theta = [0:45:135];
        %         for i=1:4
        %             [gm_img{i},~] = steerGauss(inten_img,theta(i),3,false);
        %             gm_img_upd{i} = abs(gm_img{i});
        %             %             gm_img_upd{i} = log(double(abs(gm_img{i}))); % gradient magnitude
        %             %             gm_img_upd{i}(gm_img{i}==0) = 0;
        %             % smooth
        % %             h = fspecial('gaussian',25*[1,1],6);
        % %             gm_img_upd{i} = imfilter(gm_img_upd{i},h,'replicate');
        %         end
        
        %         toc
        
        %         tic
        % point cloud - reflectivity
        ref_pcd_arr = double(points(points_id,5)); % refc
        ref_pcd_arr_upd = log(ref_pcd_arr);
        ref_pcd_arr_upd(ref_pcd_arr==0) = 0;
        % point cloud - normal
        nx_pcd_arr = double(points(points_id,6));
        ny_pcd_arr = double(points(points_id,7));
        nz_pcd_arr = double(points(points_id,8));
        % point cloud - curvature
        curv_pcd_arr = double(points(points_id,9)); % cur
        %         toc
        
        %         tic
        %%
        grid_point_num = zeros(para.grid_row_num, para.grid_col_num);
        grid_point_area = zeros(para.grid_row_num, para.grid_col_num);
        entropy_joint = zeros(nbins, nbins, para.grid_row_num, para.grid_col_num);
        for sn = 1 : sm_num
            % compute sm for each pair of attributes
            switch para.sm_list{sn}
                case 'Inten-Ref'
                    img_attr = im2double(inten_img);
                    %                     img_attr = inten_img/max(inten_img(:)); % normalize to [0,1]
                    pcd_attr = double(ref_pcd_arr_upd);
                case 'Inten-Nx'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nx_pcd_arr);
                case 'Inten-Ny'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(ny_pcd_arr);
                case 'Inten-Nz'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nz_pcd_arr);
                    
                case 'Illu1-Ref'
                    img_attr = im2double(illu_img{1});
                    %                     img_attr = inten_img/max(inten_img(:)); % normalize to [0,1]
                    pcd_attr = double(ref_pcd_arr_upd);
                case 'Illu2-Ref'
                    img_attr = im2double(illu_img{2});
                    pcd_attr = double(ref_pcd_arr_upd);
                case 'Illu3-Ref'
                    img_attr = im2double(illu_img{3});
                    pcd_attr = double(ref_pcd_arr_upd);
                    
                case 'GradM-Curv'
                    img_attr = im2double(gm_img_upd);
                    pcd_attr = double(curv_pcd_arr);
                case 'GradX-Curv'
                    img_attr = im2double(gx_img_upd);
                    pcd_attr = double(curv_pcd_arr);
                case 'GradY-Curv'
                    img_attr = im2double(gy_img_upd);
                    pcd_attr = double(curv_pcd_arr);
                    %                 case 'Grad2-Curv'
                    %                     img_attr = im2double(gm_img_upd{2});
                    %                     pcd_attr = double(curv_pcd_arr);
                    %                 case 'Grad3-Curv'
                    %                     img_attr = im2double(gm_img_upd{3});
                    %                     pcd_attr = double(curv_pcd_arr);
                    %                 case 'Grad4-Curv'
                    %                     img_attr = im2double(gm_img_upd{4});
                    %                     pcd_attr = double(curv_pcd_arr);
            end
            img_attr = img_attr(:)';
            img_attr = img_attr(v + (u-1)*para.cam.height)';
            %     valid_idx = points_idx &((u>=1 & u<=img_w) & (v>=1 & v<=img_h));
            
            % get histgram and probobility of gray, refc, joint
            % get histogram for each grid
            for i = 1:para.grid_row_num
                for j = 1:para.grid_col_num
                    % points for grid[i][j]
                    grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
                    % get area percentage
                    if sum(grid_points_idx)==0
                        grid_point_area(i, j) = 0;
                    else
                        grid_point_area(i, j) = ((max(u(grid_points_idx))-min(u(grid_points_idx)))...
                            *(max(v(grid_points_idx))-min(v(grid_points_idx))))...
                            /((grid_x(j+1)-grid_x(j))*(grid_y(i+1)-grid_y(i)))*100;
                    end
                    [~, MI_prob] = get_histogram(img_attr(grid_points_idx), pcd_attr(grid_points_idx), nbins);
                    [sm_grid(img_idx, iter_no, sn, i, j), grid_point_num(i, j), entropy_joint(:,:,i, j)]...
                        = get_NMI(MI_prob);
                end
            end
            %
            [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
            [sm_arr(img_idx, iter_no, sn), ~, ~] = get_NMI(MI_prob);
            
            if show_img
                tmp_sm_grid(:,:,:) = sm_grid(img_idx, iter_no, :, :, :);
                tmp_sm_arr = sm_arr(img_idx, iter_no, :);
                imageSave(image, para, grid_x, grid_y, u, v, ...
                    tmp_sm_grid, grid_point_num, grid_point_area, dir_frame, ...
                    inten_img, illu_img, gm_img_upd, gx_img_upd, gy_img_upd, ...
                    ref_pcd_arr_upd, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr);
            end
            
        end
        %         toc
        
%         if show_img && err_heading == 0
%             tmp_sm_grid(:,:,:) = sm_grid(img_idx, iter_no, :, :, :);
%             tmp_sm_arr = sm_arr(img_idx, iter_no, :);
%             imageSave(image, para, grid_x, grid_y, u, v, ...
%                 tmp_sm_grid, grid_point_num, grid_point_area, dir_frame, ...
%                 inten_img, illu_img, gm_img_upd, gx_img_upd, gy_img_upd, ...
%                 ref_pcd_arr_upd, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr);
%         end
    end
    
    %%
    % img_idx, iter_no, sn, i, j
    cur_sm_arr(:, :) = sm_arr(img_idx, :, :);
    cur_sm_grid(:, :, :, :) = sm_grid(img_idx, :, :, :, :);
    file_name = sprintf('SM_frame_%4d', frame_no);
    save([para.dir_output, dir_frame, file_name '.mat'], 'cur_sm_arr', 'cur_sm_grid');
    
    figure;
    plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_arr(:, 1:4), '-o');
    hold on;
    plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_arr(:, 5:7), '-x');
    plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_arr(:, 8:10), '-.');
    ylim([0.9 1.3]);
    legend(para.sm_list);
    %     set(gcf, 'PaperPositionMode', 'auto')
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
    %     maximize(gcf)
    file_name = sprintf('headingErr_trend_frame_%4d_total', frame_no);
    saveas(gcf, [para.dir_output, dir_frame, file_name]);
    %     export_fig( gcf, ...
    %         [para.dir_output, dir_frame, file_name], ...
    %         '-painters', '-jpg', '-r300' );
    close all;
    %
    for i = 1:para.grid_row_num
        for j = 1:para.grid_col_num
            figure;
            %     plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), tmp_arr(:, 1:3), '-.');
            plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_grid(:, 1:4, i, j), '-o');
            hold on;
            plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_grid(:, 5:7, i, j), '-x');
            plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), cur_sm_grid(:, 8:10, i, j), '-.');
            ylim([0.9 1.3]);
            legend(para.sm_list);
            %             set(gcf, 'PaperPositionMode', 'auto')
            set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
            %             maximize(gcf)
            file_name = sprintf('headingErr_trend_frame_%4d_grid_%d%d', frame_no, i, j);
            %             export_fig( gcf, ...
            %                 [para.dir_output, dir_frame, file_name], ...
            %                 '-painters', '-jpg', '-r300' );
            saveas(gcf, [para.dir_output, dir_frame, file_name]);
            close all;
        end
    end
end

%% average over all images
avg_sm_arr(:, :) = mean(sm_arr(:, :, :), 1);
avg_sm_grid(:, :, :, :) = mean(sm_grid(:, :, :, :, :), 1);
save([para.dir_output 'SM_average.mat'], 'avg_sm_arr', 'avg_sm_grid');

figure;
plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 1:4), '-o');
hold on;

plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 5:7), '-x');
plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 8:10), '-.');
ylim([0.9 1.3]);
legend(para.sm_list);
% set(gcf, 'PaperPositionMode', 'auto')
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
file_name = sprintf('headingErr_trend_average_total.jpg');
% maximize(gcf)
% export_fig( gcf, ...
%     [para.dir_output, file_name], ...
%     '-painters', '-jpg', '-r300' );
saveas(gcf, [para.dir_output, file_name]);
close all;
%
for i = 1:para.grid_row_num
    for j = 1:para.grid_col_num
        figure;
        plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 1:4, i, j), '-o');
        hold on;
        plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 5:7, i, j), '-x');
        plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 8:10, i, j), '-.');
        ylim([0.9 1.3]);
        legend(para.sm_list);
%         set(gcf, 'PaperPositionMode', 'auto')
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        file_name = sprintf('headingErr_trend_average_grid_%d%d.jpg', i, j);
%         maximize(gcf)
%         export_fig( gcf, ...
%             [para.dir_output, file_name], ...
%             '-painters', '-jpg', '-r300' );
        saveas(gcf, [para.dir_output, file_name]);
        close all;
    end
end
