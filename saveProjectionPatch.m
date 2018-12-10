function saveProjectionPatch(image, para, grid_x, grid_y, u, v, ...
    dir_frame, ...
    ref_pcd_arr)
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
            ref_pcd_arr(grid_points_idx)];
        %
        patch_image = image.image(grid_y(i)+ybias:grid_y(i+1)+ybias-1, ...
            grid_x(j)+xbias:grid_x(j+1)+xbias-1, :);
        close all;
        %
        figure;
        imagesc(patch_image);
        hold on
        scatter((patch_points(:, 1)), ...
            (patch_points(:, 2)), ...
            3*ones(1,size(patch_points, 1)), (patch_points(:, 3)), 'filled');
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 12 9])
        %     end
        file_name = sprintf('Patch_%d%d.jpg', i, j);
        saveas(gcf, [para.dir_output, dir_frame, file_name]);
        close all;
        clear patch_points patch_image grid_points_idx
    end
end
