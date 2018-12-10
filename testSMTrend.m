function [sm_grid] = testSMTrend(image, points, para)

show_img = true

% t, inv_R: lidar to cam
% R = makeRotationMatrix(para.x0(4:6));
% inv_R = inv(R);
% t = para.x0(1:3)';

sm_num = length(para.sm_list);
% frame_num = length(img_list);
nbins = round(256/para.bin_fraction);
% grid_x = para.ROI_width(1) : para.grid_width : para.ROI_width(2)+1;
% grid_x(end) = para.ROI_width(2)+1;
% grid_y = para.ROI_height(1) : para.grid_height : para.ROI_height(2)+1;
% grid_y(end) = para.ROI_height(2)+1;

step_num = 20;

tmp_x0 = para.x0;
% sm_min = 0.8;
% sm_max = 1.5;
sm_min = 0;
sm_max = 0.15;

% (img_idx, iter_no, sn, i, j)
sm_grid = zeros(2*step_num+1, sm_num, para.grid_row_num, para.grid_col_num);
sm_arr = zeros(2*step_num+1, sm_num);

% for img_idx = 1:frame_num
%     image = img_list{img_idx};
frame_no = image.frame_no
R_w2v = makeRotationMatrix(image.prh);
inv_R_w2v = inv(R_w2v);
%
dir_frame = ['frame_' num2str(frame_no) '\'];
mkdir([para.dir_output dir_frame]);

h = fspecial('gaussian',11*[1,1],6);
%         tic
% image - intensity
inten_img = rgb2gray(image.image);
inten_img_eq = adapthisteq(inten_img);
% image - illumination inv
tmp_img = shadowremoval(im2double(image.image), 'ycbcr');
%             tmp_img = rgb2ycbcr(double(image.image));
y_img = tmp_img(:,:,1);
cb_img = tmp_img(:,:,2);
cr_img = tmp_img(:,:,3);
tmp_img = rgb2hsv(double(image.image));
h_img = tmp_img(:,:,1);
s_img = tmp_img(:,:,2);
v_img = tmp_img(:,:,3);
% image - gradient
[gm_img_int, gd_img_int] = imgradient(inten_img);
[gm_img_inteq, gd_img_inteq] = imgradient(inten_img_eq);
[gm_img_hue, gd_img_hue] = imgradient(h_img);
% gm_img_eq = adapthisteq(mat2gray(gm_img));
% gd_img_eq = adapthisteq(mat2gray(gd_img));
%             gm_img_upd = log(double(gm_img)); % gradient magnitude
%             gm_img_upd(gm_img==0) = 0;
% gm_img_fil = imfilter(gm_img,h,'replicate');
% gd_img_fil = imfilter(gd_img,h,'replicate');
edge_img_int = edge(inten_img, 'Canny', [0.05, 0.3]);
% edge_img_inteq = edge(inten_img_eq, 'Canny', [0.05, 0.3]);
% edge_img_hue = edge(h_img, 'Canny', [0.05, 0.3]);

%     iter_no = 1;
for eno = 1:length(para.err_list)
    err_frame = ['error_' para.err_list{eno} '\'];
    mkdir([para.dir_output dir_frame err_frame]);
    for iter_no = 1:1+2*step_num
        %% add error to registration parameters
        [t, R, inv_R, err_val] = addError(para.err_list{eno}, tmp_x0, iter_no, step_num);
        %         tic
        [u, v, points_id] = imageProjection_upd(points, image, para, t, R, inv_R);
        %         [u, v, points_id] = imageProjection(points, image, para, t, R, inv_R);
        %         tic
        % point cloud - reflectivity
        ref_pcd_arr = double(points(points_id,5)); % refc
        %             ref_pcd_arr_upd = log(ref_pcd_arr);
        %             ref_pcd_arr_upd(ref_pcd_arr==0) = 0;
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
curv_pcd_arr = (curv_pcd_arr - min(curv_pcd_arr)) / (max(curv_pcd_arr) - min(curv_pcd_arr));
%         curv_pcd_arr_eq = histeq(curv_pcd_arr);
        % point cloud - edge
%         edge_pcd_arr = double(points(points_id,11));
gm_ref_arr = double(points(points_id,11));
gm_ref_arr = (gm_ref_arr - min(gm_ref_arr)) / (max(gm_ref_arr) - min(gm_ref_arr));
gm_rng_arr = double(points(points_id,12));
gm_rng_arr = (gm_rng_arr - min(gm_rng_arr)) / (max(gm_rng_arr) - min(gm_rng_arr));
        % combine
        %         max_curv = max(curv_pcd_arr);
        %         curv_pcd_arr = curv_pcd_arr + edge_pcd_arr * max_curv;
%         max_curv = max(curv_pcd_arr);
%         curv_pcd_arr_comb = curv_pcd_arr + edge_pcd_arr * max_curv;
curv_comb_arr = curv_pcd_arr + gm_rng_arr;
grad_comb_arr = curv_comb_arr + gm_ref_arr;
        %         toc
        
        %         tic
        %%
        %             grid_point_num = zeros(para.grid_row_num, para.grid_col_num);
        %             grid_point_area = zeros(para.grid_row_num, para.grid_col_num);
        %             entropy_joint = zeros(nbins, nbins, para.grid_row_num, para.grid_col_num);
        for sn = 1 : sm_num
            % compute sm for each pair of attributes
            switch para.sm_list{sn}
                    %
                case 'Inten-Ref'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(ref_pcd_arr);
                case 'IntenEQ-Ref'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(ref_pcd_arr);
                case 'Y-Ref'
                    img_attr = im2double(y_img);
                    pcd_attr = double(ref_pcd_arr);
                case 'H-Ref'
                    img_attr = im2double(h_img);
                    pcd_attr = double(ref_pcd_arr);
                    %
                case 'Inten-Nx'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nx_pcd_arr);
                case 'Inten-Ny'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(ny_pcd_arr);
                case 'Inten-Nz'
                    img_attr = im2double(inten_img);
                    pcd_attr = double(nz_pcd_arr);
                    %
                case 'IntenEQ-Nx'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(nx_pcd_arr);
                case 'IntenEQ-Ny'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(ny_pcd_arr);
                case 'IntenEQ-Nz'
                    img_attr = im2double(inten_img_eq);
                    pcd_attr = double(nz_pcd_arr);
                    %
                case 'GradMInt-CurvComb'
                    img_attr = im2double(gm_img_int);
                    pcd_attr = double(curv_comb_arr);
                case 'GradMHue-GradMRef'
                    img_attr = im2double(gm_img_hue);
                    pcd_attr = double(gm_ref_arr);
                case 'GradMHue-GradMComb'
                    img_attr = im2double(gm_img_hue);
                    pcd_attr = double(grad_comb_arr);
                    %
            end
            img_attr = img_attr(:)';
            img_attr = img_attr(v + (u-1)*para.cam.height)';
            %     valid_idx = points_idx &((u>=1 & u<=img_w) & (v>=1 & v<=img_h));
            
            % get histgram and probobility of gray, refc, joint
            % get histogram for each grid
            %             for i = 1:para.grid_row_num
            %                 parfor j = 1:para.grid_col_num
            %                     % points for grid[i][j]
            %                     grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
            %                     % get area percentage
            %                     if sum(grid_points_idx)==0
            %                         grid_point_area(i, j) = 0;
            %                     else
            %                         grid_point_area(i, j) = ((max(u(grid_points_idx))-min(u(grid_points_idx)))...
            %                             *(max(v(grid_points_idx))-min(v(grid_points_idx))))...
            %                             /((grid_x(j+1)-grid_x(j))*(grid_y(i+1)-grid_y(i)))*100;
            %                     end
            %                     [~, MI_prob] = get_histogram(img_attr(grid_points_idx), pcd_attr(grid_points_idx), nbins);
            %                     [sm_grid(iter_no, sn, i, j), grid_point_num(i, j), entropy_joint(:,:,i, j)]...
            %                         = get_NMI(MI_prob, nbins);
            %                     %                         [sm_grid(iter_no, sn, i, j), ~, ~] = get_NMI(MI_prob, nbins);
            %                 end
            %             end
            %
            [~, MI_prob] = get_histogram(img_attr, pcd_attr, nbins);
            [sm_arr(iter_no, sn), ~, ~] = get_NMI(MI_prob);
        end
        %         toc
        if show_img %&& err_heading == 0
            %             tmp_sm_grid(:,:,:) = sm_grid(iter_no, :, :, :);
            %                 tmp_sm_arr = sm_arr(img_idx, iter_no, :);
            %                 saveProjectionImage(image, para, grid_x, grid_y, u, v, ...
            %                     tmp_sm_grid, grid_point_num, grid_point_area, [dir_frame err_frame], ...
            %                     inten_img, ref_pcd_arr_eq, para.err_list{eno}, err_val);
            if iter_no == 1 + step_num
                %                 saveFeaturePatch(image, para, grid_x, grid_y, u, v, ...
                %                     [dir_frame err_frame], ...
                %                     inten_img, inten_img_eq, y_img, h_img, gd_img_hue, gm_img_hue, ...
                %                     ref_pcd_arr_eq, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr_eq);
                %                 saveProjectionPatch(image, para, grid_x, grid_y, u, v, ...
                %                     [dir_frame err_frame], ...
                %                     ref_pcd_arr_eq);
                %                 saveFeatureImage(image, para, u, v, [dir_frame], ...
                %                     inten_img, inten_img_eq, y_img, h_img, gd_img_hue, gm_img_hue, ...
                %                     ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr_eq);
                saveFeatureImage(image, para, u, v, [dir_frame], ...
                    inten_img, inten_img_eq, y_img, h_img, gm_img_hue, ...
                    ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_comb_arr, gm_ref_arr, grad_comb_arr);
                saveProjectionImage(image, para, u, v, ...
                    [dir_frame], ...
                    inten_img, ref_pcd_arr);
            end
        end
    end
    %%
    % img_idx, iter_no, sn, i, j
    cur_sm_arr(:, :) = sm_arr(:, :);
    %     cur_sm_grid(:, :, :, :) = sm_grid(:, :, :, :);
    file_name = sprintf('SM_frame_%04d_%s', frame_no, para.err_list{eno});
    save([para.dir_output, dir_frame, err_frame, file_name '.mat'], 'cur_sm_arr');
    
    figure;
    min_val = min(cur_sm_arr(:, :), [], 1);
    plot(-step_num : step_num, cur_sm_arr(:, 1:3) - repmat(min_val(1:3), size(cur_sm_arr, 1), 1), '-o');
    hold on;
    plot(-step_num : step_num, cur_sm_arr(:, 4:5) - repmat(min_val(4:5), size(cur_sm_arr, 1), 1), '-x');
%     plot(-step_num : step_num, cur_sm_arr(:, 7:9) - repmat(min_val(7:9), size(cur_sm_arr, 1), 1), '-.');
    %     plot(-step_num : step_num, cur_sm_arr(:, 9:10) - repmat(min_val(9:10), size(cur_sm_arr, 1), 1), '-');
    %     ylim([sm_min sm_max]);
    legend(para.sm_list);
    %     set(gcf, 'PaperPositionMode', 'auto')
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
    %     maximize(gcf)
    file_name = sprintf('Trend_frame_%04d_%s_total', para.err_list{eno}, frame_no);
    saveas(gcf, [para.dir_output, dir_frame, err_frame, file_name]);
    %     export_fig( gcf, ...
    %         [para.dir_output, dir_frame, file_name], ...
    %         '-painters', '-jpg', '-r300' );
    close all;
    %
    %     for i = 1:para.grid_row_num
    %         for j = 1:para.grid_col_num
    %             figure;
    %             min_val = min(cur_sm_grid(:, :, i, j), [], 1);
    %             %     plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), tmp_arr(:, 1:3), '-.');
    %             plot(-step_num : step_num, cur_sm_grid(:, 1:4, i, j) - repmat(min_val(1:4), size(cur_sm_grid, 1), 1), '-o');
    %             hold on;
    %             plot(-step_num : step_num, cur_sm_grid(:, 5:7, i, j) - repmat(min_val(5:7), size(cur_sm_grid, 1), 1), '-x');
    %             plot(-step_num : step_num, cur_sm_grid(:, 8:9, i, j) - repmat(min_val(8:9), size(cur_sm_grid, 1), 1), '-.');
    %             %                 plot(-step_num : step_num, cur_sm_grid(:, 19:22, i, j), '-');
    %             %             ylim([sm_min sm_max]);
    %             legend(para.sm_list);
    %             %             set(gcf, 'PaperPositionMode', 'auto')
    %             set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
    %             %             maximize(gcf)
    %             file_name = sprintf('Trend_frame_%4d_%s_grid%d%d', frame_no, para.err_list{eno}, i, j);
    %             %             export_fig( gcf, ...
    %             %                 [para.dir_output, dir_frame, file_name], ...
    %             %                 '-painters', '-jpg', '-r300' );
    %             saveas(gcf, [para.dir_output, dir_frame, err_frame, file_name]);
    %             close all;
    %         end
    %     end
end
% end

% %% average over all images
% avg_sm_arr(:, :) = mean(sm_arr(:, :, :), 1);
% avg_sm_grid(:, :, :, :) = mean(sm_grid(:, :, :, :, :), 1);
% save([para.dir_output 'SM_average.mat'], 'avg_sm_arr', 'avg_sm_grid');
%
% figure;
% plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 1:4), '-o');
% hold on;
% plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 5:10), '-x');
% plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_arr(:, 11:12), '-.');
% ylim([sm_min sm_max]);
% legend(para.sm_list);
% % set(gcf, 'PaperPositionMode', 'auto')
% set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
% file_name = sprintf('headingErr_trend_average_total.jpg');
% % maximize(gcf)
% % export_fig( gcf, ...
% %     [para.dir_output, file_name], ...
% %     '-painters', '-jpg', '-r300' );
% saveas(gcf, [para.dir_output, file_name]);
% close all;
% %
% for i = 1:para.grid_row_num
%     for j = 1:para.grid_col_num
%         figure;
%         plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 1:4, i, j), '-o');
%         hold on;
%         plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 5:10, i, j), '-x');
%         plot(d_heading * (-step_num) : d_heading : d_heading * (step_num), avg_sm_grid(:, 11:12, i, j), '-.');
%         ylim([sm_min sm_max]);
%         legend(para.sm_list);
%         %         set(gcf, 'PaperPositionMode', 'auto')
%         set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
%         file_name = sprintf('headingErr_trend_average_grid_%d%d.jpg', i, j);
%         %         maximize(gcf)
%         %         export_fig( gcf, ...
%         %             [para.dir_output, file_name], ...
%         %             '-painters', '-jpg', '-r300' );
%         saveas(gcf, [para.dir_output, file_name]);
%         close all;
%     end
% end
