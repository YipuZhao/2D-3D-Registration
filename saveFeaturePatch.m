function saveFeaturePatch(image, para, grid_x, grid_y, u, v, dir_frame, ...
    inten_img, inten_img_eq, y_img, h_img, h_gd_img, h_gm_img, ...
    ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr)

frame_no = image.frame_no;
attr_list = {
    'int';
    'int_{hist}';
    'int_{shadow}';
    'hue';
    'grad^{hue}_{dir}';
    'grad^{hue}_{mag}';
    %
    'ref';
    'norm_x';
    'norm_y';
    'norm_z';
    'curv';
    };

points_thres = 1000;
for i = 1:para.grid_row_num
    for j = 1:para.grid_col_num
        xbias = -para.ROI_width(1)+1;
        ybias = -para.ROI_height(1)+1;
        % points for grid[i][j]
        grid_points_idx = ((u>=grid_x(j) & u<grid_x(j+1)) & (v>=grid_y(i) & v<grid_y(i+1)));
        %                 ref_pcd_arr = log(points(points_id(grid_points_idx), 5));
        %                 ref_pcd_arr(ref_pcd_arr==0) = 0;
        %                 ref_pcd_arr = points(points_id(grid_points_idx), 6);
        % remove 2D patches without enough 3D points projected
        if length(ref_pcd_arr(grid_points_idx)) < points_thres
            continue ;
        end
        patch_points(:, :) = [u(grid_points_idx)'+xbias-grid_x(j), ...
            v(grid_points_idx)'+ybias-grid_y(i), ...
            ref_pcd_arr(grid_points_idx), ...
            nx_pcd_arr(grid_points_idx), ...
            ny_pcd_arr(grid_points_idx), ...
            nz_pcd_arr(grid_points_idx), ...
            curv_pcd_arr(grid_points_idx)
            ];
        %
        patch_image{1} = inten_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        patch_image{2} = inten_img_eq(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        patch_image{3} = y_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        patch_image{4} = h_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
%         patch_image{5} = int_gd_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
%             grid_x(j)+xbias:grid_x(j+1)+xbias-1);
%         patch_image{6} = inteq_gd_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
%             grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        patch_image{5} = h_gd_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        patch_image{6} = h_gm_img(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1);
        %
        figure;
        for k=1:6
            subplot(2, 6, k);
            imagesc(patch_image{k});
            colormap jet;
            title(attr_list{k});
        end
        for k=7:11
            subplot(2, 6, k);
            imagesc(repmat(patch_image{1},[1,1,3]));
            hold on;
            scatter(patch_points(:, 1), patch_points(:, 2), ...
                3*ones(1,length(patch_points)), patch_points(:, k-4), 'filled');
            colormap jet;
            title(attr_list{k});
        end

        %%
        file_name = sprintf('Attribute_%d%d', i, j);
        %             title(gcf, file_name);
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 36 12])
        %             set(gcf, 'PaperPositionMode', 'auto')
        %             maximize(gcf)
        %             export_fig( gcf, ...
        %                 [para.dir_output, dir_frame, file_name], ...
        %                 '-painters', '-jpg', '-r300' );
        
        saveas(gcf, [para.dir_output, dir_frame, file_name '.jpg']);
        close all
        clear patch_points patch_image grid_points_idx
    end
end