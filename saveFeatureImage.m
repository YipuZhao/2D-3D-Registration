function saveFeatureImage(image, para, u, v, dir_frame, ...
   inten_img, inten_img_eq, y_img, h_img, gm_img_hue, ...
   ref_pcd_arr, nx_pcd_arr, ny_pcd_arr, nz_pcd_arr, curv_comb_arr, gm_ref_arr, grad_comb_arr)

frame_no = image.frame_no;
attr_list = {
    'int';
    'int_{hist}';
    'int_{shadow}';
    'hue';
    'grad^{hue}_{mag}';
    %
    'ref';
    'norm_x';
    'norm_y';
    'norm_z';
    'curv';
    'grad^{ref}_{mag}';
    'grad^{comb}_{mag}';
    };

figure_points(:, :) = [
    u', ...
    v', ...
    ref_pcd_arr, ...
    nx_pcd_arr, ...
    ny_pcd_arr, ...
    nz_pcd_arr, ...
    curv_comb_arr, ...
    gm_ref_arr, ...
    grad_comb_arr
    ];
%
figure_image{1} = inten_img;
figure_image{2} = inten_img_eq;
figure_image{3} = y_img;
figure_image{4} = h_img;
figure_image{5} = gm_img_hue;

figure;
for k=1:5
    subplot(2, 7, k);
    imagesc(figure_image{k});
    colormap parula;
    title(attr_list{k});
end
for k=8:14
    subplot(2, 7, k);
    imagesc(repmat(figure_image{1},[1,1,3]));
    hold on;
    scatter(figure_points(:, 1), figure_points(:, 2), ...
        ones(1,length(figure_points)), figure_points(:, k-5), 'filled');
    colormap parula;
    title(attr_list{k-2});
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
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 56 12])
%             set(gcf, 'PaperPositionMode', 'auto')
%             maximize(gcf)
%             export_fig( gcf, ...
%                 [para.dir_output, dir_frame, file_name], ...
%                 '-painters', '-jpg', '-r300' );

saveas(gcf, [para.dir_output, dir_frame, file_name '.jpg']);
close all