function [mi, combine_mi] = save_segment_test(file_head, para, frame_no, u, v, image_gray, points_pts, points_seg, mi_cost, range)
mi_cost_core = mi_cost(floor((size(mi_cost,1)+1)/2),:);
figure;
subplot_tight(1,6,4:6,.05);
% save image and points could projection
ROI_h = para.ROI_height(2)-para.ROI_height(1);
ROI_w = para.ROI_width(2)-para.ROI_width(1);
xbias = -para.ROI_width(1)+1;
ybias = -para.ROI_height(1)+1;

image_ROI = image_gray(para.ROI_height(1):para.ROI_height(2), para.ROI_width(1):para.ROI_width(2));
imagesc(repmat(image_ROI,[1,1,3]));
hold on
c = my_colormap(points_seg);
% points_pts = points_pts*1.4;
% points_pts(points_pts>1) = 1;
% c = [points_pts.*c(:,1), points_pts.*c(:,2), points_pts.*c(:,3)];
scatter(fliplr(u+xbias), fliplr(v+ybias), 3*ones(1,length(u)), flipud(c), 'filled');
c = my_colormap([mi_cost_core.id]);
for i = 1:size(c,1)
    cu = mean(u(points_seg==mi_cost_core(i).id));
    cv = mean(v(points_seg==mi_cost_core(i).id));
    s = sprintf('%d', mi_cost_core(i).id);
    text(cu, cv, s, 'Color', c(i,:), 'FontSize', 15, 'FontWeight', 'bold');
end


subplot_tight(1,6,1,.05);
% put text 'mi_cost \n (points_num, points_area)' in each grid
c = my_colormap([mi_cost_core.id]);
y = 0;
for i = 1:size(c,1)
    s = sprintf('%d: %0.6f / %d', mi_cost_core(i).id, mi_cost_core(i).mi, mi_cost_core(i).num);
    text(0, y, s, 'Color', c(i,:), 'FontSize', 20, 'FontWeight', 'bold');
    y = y-20;
end
ylim([y,20]);

subplot_tight(1,6,2:3,.05);
mi = reshape([mi_cost.mi],size(mi_cost,1),[]);
H = reshape([mi_cost.H],size(mi_cost,1),[]);
% mi_mean = mi-repmat(mean(mi,1),[size(mi,1),1]);
x = ((1:size(mi,1))-11)*range;
hold on
for i = 1:size(mi,2)
    plot(x,mi(:,i),'Color', c(i,:));
end
combine_mi = get_combine_mi(H);
plot(x, combine_mi,'LineWidth',5, 'Color', 'red');
xlim([min(x),max(x)])
xlabel('heading bias');
ylabel('NMI')
% legend('show')

save_width = 10;
save_height = round(save_width*ROI_h/ROI_w)*1.5;
save_width = save_width*3;
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 save_width+1 save_height+1])
file_name = sprintf('%s_%s_%s_image_%06d.jpg',para.seg_type, para.map, file_head, frame_no);
saveas(gcf,[para.dir_output, file_name]);
% data_name = sprintf('%s_%s_%s_image_%06d.mat',para.seg_type, para.map, file_head, frame_no);
% save(data_name,'mi','combine_mi');
close all;