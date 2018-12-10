function saveProjectionImage(image, para, u, v, ...
            dir_frame, ...
            inten_img, ref_pcd_arr)
% function saveProjectionImage(image, para, grid_x, grid_y, u, v, ...
%             dir_frame, ...
%             inten_img, ref_pcd_arr, err_type, err_value)


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
% for i = 2:length(grid_x)-1
%     plot([1,1]*grid_x(i)+xbias, [1,para.cam.height]+ybias, 'Color', 'red', 'LineWidth', 3);
% end
% for i = 2:length(grid_y)-1
%     plot([1,para.cam.width]+xbias, [1,1]*grid_y(i)+ybias, 'Color', 'red', 'LineWidth', 3);
% end
% put text 'mi_cost \n (points_num, points_area)' in each grid
% for i = 1:para.grid_row_num
%     for j = 1:para.grid_col_num
%         s = sprintf('%0.6f', sm_grid(1, i, j));
%         text(grid_x(j)+xbias, grid_y(i+1)-0.075*(para.ROI_height(2)-para.ROI_height(1))+ybias, s, 'Color', 'yellow', 'FontSize', 20, 'FontWeight', 'bold');
%         s = sprintf('(%d, %0.3f)', grid_point_num(i, j), grid_point_area(i, j));
%         text(grid_x(j)+xbias, grid_y(i+1)-0.025*(para.ROI_height(2)-para.ROI_height(1))+ybias, s, 'Color', 'red', 'FontSize', 20, 'FontWeight', 'bold');
%     end
% end
% title(['fr ' num2str(frame_no) ' ' err_type ' ' num2str(err_value)]);
save_width = 16;
save_height = round(save_width*(para.ROI_height(2)-para.ROI_height(1))/(para.ROI_width(2)-para.ROI_width(1)));
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 save_width+1 save_height+1])
% file_name = sprintf('projectionImage_%s_%.03f.jpg', err_type, err_value);
file_name = sprintf('Patch.jpg');
saveas(gcf,[para.dir_output, dir_frame, file_name]);
close all;
