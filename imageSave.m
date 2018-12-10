function imageSave(image, para, grid_x, grid_y, u, v, ...
            sm_grid, grid_point_num, grid_point_area, dir_frame, ...
            inten_img, illu_img, gm_img_upd, gx_img_upd, gy_img_upd, ...
            ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_pcd_arr)

% figure;
% % grid MI calculation
% for i = 1:para.grid_row_num
%     for j = 1:para.grid_col_num
%         subplot(para.grid_row_num, para.grid_col_num, (i-1)*para.grid_col_num + j);
%         imagesc(abs(entropy_joint(:,:,i,j)));
%         colorbar
%         axis equal;
%         xlim([1,64]);
%         ylim([1,64]);
%         xlabel('gray');
%         ylabel('refc');
%         title([num2str(grid_sm(1,i,j)), ' & ', num2str(grid_sm(8,i,j)), ' / ', num2str(grid_point_num(i,j))]);
%     end
% end
% % set prob_joint figure size and save
% set(gcf,'PaperUnits','inches','PaperPosition',[0 0 24 12])
% file_name = sprintf('jointentropy_%04d.jpg', image.frame_no);
% print('-djpeg', [para.dir_output, dir_frame, file_name], '-r100');

frame_no = image.frame_no;

% save image and points could projection
figure;
image_ROI = inten_img(para.ROI_height(1):para.ROI_height(2), para.ROI_width(1):para.ROI_width(2));
imagesc(repmat(image_ROI,[1,1,3]));
hold on
xbias = -para.ROI_width(1)+1;
ybias = -para.ROI_height(1)+1;
scatter(fliplr(u+xbias), fliplr(v+ybias), 3*ones(1,length(u)), flipud(ref_pcd_arr), 'filled');
% draw grid frames
for i = 2:length(grid_x)-1
    plot([1,1]*grid_x(i)+xbias, [1,para.cam.height]+ybias, 'Color', 'red', 'LineWidth', 3);
end
for i = 2:length(grid_y)-1
    plot([1,para.cam.width]+xbias, [1,1]*grid_y(i)+ybias, 'Color', 'red', 'LineWidth', 3);
end
% put text 'mi_cost \n (points_num, points_area)' in each grid
for i = 1:para.grid_row_num
    for j = 1:para.grid_col_num
        s = sprintf('%0.6f', sm_grid(1, i, j));
        text(grid_x(j)+xbias, grid_y(i+1)-0.075*(para.ROI_height(2)-para.ROI_height(1))+ybias, s, 'Color', 'yellow', 'FontSize', 20, 'FontWeight', 'bold');
        s = sprintf('(%d, %0.3f)', grid_point_num(i, j), grid_point_area(i, j));
        text(grid_x(j)+xbias, grid_y(i+1)-0.025*(para.ROI_height(2)-para.ROI_height(1))+ybias, s, 'Color', 'red', 'FontSize', 20, 'FontWeight', 'bold');
    end
end
save_width = 16;
save_height = round(save_width*(para.ROI_height(2)-para.ROI_height(1))/(para.ROI_width(2)-para.ROI_width(1)));
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 save_width+1 save_height+1])
file_name = sprintf('gridvisualization_%04d.jpg', frame_no);
saveas(gcf,[para.dir_output, dir_frame, file_name]);
close all;

% if err_heading == 0
%             %%
%             close all;
%             if show_img
figure;
subplot(3, 4, 1);
imagesc(inten_img);
colormap jet;
title(['image intensity fr' num2str(frame_no)]);
%
for i=1:3
    subplot(3, 4, 1+i);
    imagesc(illu_img{i});
    colormap jet;
    title(['image illumination col ' num2str(i) ' fr' num2str(frame_no)]);
end
%
%                 for i=1:4
%                     subplot(3, 5, 4+i);
%                     imagesc(gm_img_upd{i});
%                     colormap jet;
%                     title(['image gradient direction ' num2str(i) ' fr' num2str(frame_no)]);
%                 end
subplot(3, 4, 5);
imagesc(gm_img_upd);
colormap jet;
title(['image gradient magnitude fr' num2str(frame_no)]);
subplot(3, 4, 6);
imagesc(gx_img_upd);
colormap jet;
title(['image gradient X fr' num2str(frame_no)]);
subplot(3, 4, 7);
imagesc(gy_img_upd);
colormap jet;
title(['image gradient Y fr' num2str(frame_no)]);
%
subplot(3, 4, 8);
imagesc(image.image);
hold on;
scatter(u, v, 3*ones(1,length(u)), ref_pcd_arr, 'filled');
colormap jet;
%             set(gca,'CLim',[0 255]);
title(['lidar reflectivity fr' num2str(frame_no)]);
%
subplot(3, 4, 9);
imagesc(image.image);
hold on;
scatter(u, v, 3*ones(1,length(u)), nx_pcd_arr, 'filled');
colormap jet;
%             set(gca,'CLim',[0 255]);
title(['lidar normal\_x fr' num2str(frame_no)]);
%
subplot(3, 4, 10);
imagesc(image.image);
hold on;
scatter(u, v, 3*ones(1,length(u)), ny_pcd_arr, 'filled');
colormap jet;
%             set(gca,'CLim',[0 255]);
title(['lidar normal\_y fr' num2str(frame_no)]);
%
subplot(3, 4, 11);
imagesc(image.image);
hold on;
scatter(u, v, 3*ones(1,length(u)), nz_pcd_arr, 'filled');
colormap jet;
%             set(gca,'CLim',[0 255]);
title(['lidar normal\_z fr' num2str(frame_no)]);
%
subplot(3, 4, 12);
imagesc(image.image);
hold on;
scatter(u, v, 3*ones(1,length(u)), curv_pcd_arr, 'filled');
colormap jet;
%             set(gca,'CLim',[0 255]);
title(['lidar curvature fr' num2str(frame_no)]);

file_name = sprintf('headingErr_0');
%             title(gcf, file_name);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 12])
%             set(gcf, 'PaperPositionMode', 'auto')
%             maximize(gcf)
%             export_fig( gcf, ...
%                 [para.dir_output, dir_frame, file_name], ...
%                 '-painters', '-jpg', '-r300' );

saveas(gcf, [para.dir_output, dir_frame, file_name '.jpg']);
close all
%                 %                 figure;
%                 %                 subplot(2, 3, 1);
%                 %                 imshow(image.image);
%                 %                 hold on;
%                 %                 scatter(u, v, 3*ones(1,length(u)), ref_pcd_arr_upd, 'filled');
%                 %                 colormap jet;
%                 %                 %             set(gca,'CLim',[0 255]);
%                 %                 title(['reflectivity fr' num2str(frame_no)]);
%                 %                 %
%                 %                 subplot(2, 3, 2);
%                 %                 imshow(image.image);
%                 %                 hold on;
%                 %                 scatter(u, v, 3*ones(1,length(u)), curv_pcd_arr, 'filled');
%                 %                 colormap jet;
%                 %                 %             set(gca,'CLim',[0 255]);
%                 %                 title(['curvature fr' num2str(frame_no)]);
%                 %                 %
%                 %                 subplot(2, 3, 3);
%                 %                 imshow(image.image);
%                 %                 hold on;
%                 %                 scatter(u, v, 3*ones(1,length(u)), nx_pcd_arr, 'filled');
%                 %                 colormap jet;
%                 %                 %             set(gca,'CLim',[0 255]);
%                 %                 title(['normal\_x fr' num2str(frame_no)]);
%                 %                 %
%                 %                 subplot(2, 3, 4);
%                 %                 imshow(image.image);
%                 %                 hold on;
%                 %                 scatter(u, v, 3*ones(1,length(u)), ny_pcd_arr, 'filled');
%                 %                 colormap jet;
%                 %                 %             set(gca,'CLim',[0 255]);
%                 %                 title(['normal\_y fr' num2str(frame_no)]);
%                 %                 %
%                 %                 subplot(2, 3, 5);
%                 %                 imshow(image.image);
%                 %                 hold on;
%                 %                 scatter(u, v, 3*ones(1,length(u)), nz_pcd_arr, 'filled');
%                 %                 colormap jet;
%                 %                 %             set(gca,'CLim',[0 255]);
%                 %                 title(['normal\_z fr' num2str(frame_no)]);
%                 %
%                 %                 file_name = sprintf('headingErr_%.02f', err_heading);
%                 %                 %             title(gcf, file_name);
%                 %                 set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 12])
%                 %                 %             set(gcf, 'PaperPositionMode', 'auto')
%                 %                 %             maximize(gcf)
%                 %                 %             export_fig( gcf, ...
%                 %                 %                 [para.dir_output, dir_frame, file_name], ...
%                 %                 %                 '-painters', '-jpg', '-r300' );
%                 %
%                 %                 saveas(gcf, [para.dir_output, dir_frame, file_name '.jpg']);
%                 %                 close all
%             end
%         end