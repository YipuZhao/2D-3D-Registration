function saveFeatureImage_test(image, para, u, v, dir_frame, ...
    edge_img_int, gm_img_int, gm_img_inteq, gm_img_hue, ...
    edge_pcd_arr, curv_pcd_arr, curv_pcd_arr_eq, curv_pcd_arr_comb)

frame_no = image.frame_no;
attr_list = {
    'EdgeInt';
    'grad^{int}_{mag}';
    'grad^{inteq}_{mag}';
    'grad^{hue}_{mag}';
    %
    'EdgeRng';
    'Curv';
    'CurvEQ';
    'CurvComb';
    };

figure_points(:, :) = [
    u', ...
    v', ...
    edge_pcd_arr, ...
    curv_pcd_arr, ...
    curv_pcd_arr_eq, ...
    curv_pcd_arr_comb
    ];
%
figure_image{1} = edge_img_int;
figure_image{2} = gm_img_int;
figure_image{3} = gm_img_inteq;
figure_image{4} = gm_img_hue;

figure;
for k=1:4
    subplot(2, 4, k);
    imagesc(figure_image{k});
    colormap jet;
    title(attr_list{k});
end
for k=5:8
    subplot(2, 4, k);
    imagesc(image.image);
    hold on;
    scatter(figure_points(:, 1), figure_points(:, 2), ...
        ones(1,length(figure_points)), figure_points(:, k-2), 'filled');
    colormap jet;
    title(attr_list{k});
end

% figure;
% subplot(2, 6, 1);
% imagesc(inten_img);
% colormap jet;
% title(['intensity image']);
% subplot(2, 6, 2);
% imagesc(inteq_img);
% colormap jet;
% title(['intensity image (histogram equalized)']);
% subplot(2, 6, 3);
% imagesc(y_img);
% colormap jet;
% title(['intensity image (shadow removed)']);
% subplot(2, 6, 4);
% imagesc(h_img);
% colormap jet;
% title(['hue image']);
% %
% subplot(2, 6, 5);
% imagesc(h_gd_img);
% colormap jet;
% title(['gradient oriantation of hue image']);
% subplot(2, 6, 6);
% imagesc(h_gm_img);
% colormap jet;
% title(['gradient magnitude of hue image']);
% %
% subplot(2, 6, 7);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), ref_pcd_arr, 'filled');
% colormap jet;
% %             set(gca,'CLim',[0 255]);
% title(['lidar reflectivity projection']);
% %
% subplot(2, 6, 8);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), nx_pcd_arr, 'filled');
% colormap jet;
% %             set(gca,'CLim',[0 255]);
% title(['lidar normal\_x projection']);
% %
% subplot(2, 6, 9);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), ny_pcd_arr, 'filled');
% colormap jet;
% %             set(gca,'CLim',[0 255]);
% title(['lidar normal\_y projection']);
% %
% subplot(2, 6, 10);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), nz_pcd_arr, 'filled');
% colormap jet;
% %             set(gca,'CLim',[0 255]);
% title(['lidar normal\_z projection']);
% %
% subplot(2, 6, 11);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), curv_pcd_arr, 'filled');
% colormap jet;
% %             set(gca,'CLim',[0 255]);
% title(['lidar curvature projection']);
%
% subplot(3, 4, 12);
% imagesc(image.image);
% hold on;
% scatter(u, v, 3*ones(1,length(u)), edge_pcd_arr, 'filled');
% colormap jet;
% title(['lidar edge fr' num2str(frame_no)]);

%%
file_name = sprintf('Attribute');
%             title(gcf, file_name);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 48 12])
%             set(gcf, 'PaperPositionMode', 'auto')
%             maximize(gcf)
%             export_fig( gcf, ...
%                 [para.dir_output, dir_frame, file_name], ...
%                 '-painters', '-jpg', '-r300' );

saveas(gcf, [para.dir_output, dir_frame, file_name '.jpg']);
close all