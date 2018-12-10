function [points, img_list] = loadData(dir_name, dir_image_name, para)

% GPS_short
GPS_name = 'Cam_GPS.txt';
%UTC x y z h p r frameno
GPS_t = textread([dir_name para.data_name '_' GPS_name], '', 'delimiter', ' ', 'headerlines',2);

% point cloud
point_cloud_name = 'Scan_for_MI.txt';
%UTC x y z refc182 nx ny nz cur seg
points = textread([dir_name para.data_name '_' point_cloud_name], '', 'delimiter', ' ', 'headerlines',2);
% seg = textread([dir_name para.data_name '_' 'seg.csv'], '', 'delimiter', ' ');
% % refctivity remapping
% % refc = points(:, 5);
% % refc = floor(refc/5.5);
% % points(:,5) = refc;
% % add normals and curvature
% % [normals,curvature] = findPointNormals(points(:,2:4),[],[0,0,10],true);
% points = [points, seg];
% fileID = fopen([dir_name para.data_name '_' point_cloud_name],'w');
% ptc = points';
% num = size(ptc,2);
% fprintf(fileID,'%d\n',size(ptc,2));
% fprintf(fileID,'UTC x y z refc182 nx ny nz cur seg\n');
% fprintf(fileID,'%.6f %.6f %.6f %.6f %4d %.6f %.6f %.6f %.6f %d\n',ptc);
% % close all;
% fclose(fileID);
if para.pavement_cut
    points = points((points(:,10)~=0),:);
end

% % re-compute normal and curvature
% [normals,curvature]=findPointNormals(points(:,2:4), 30, [0,0,0]);
% points(:, 6:8) = normals;
% points(:, 9) = curvature;
% 
% % re-write points to file
% point_cloud_name = 'Scan_for_MI_new.txt';
% fileID = fopen([para.dir_name para.data_name '_' point_cloud_name],'w');
% ptc = points';
% num = size(ptc,2);
% fprintf(fileID,'%d\n',size(ptc,2));
% fprintf(fileID,'UTC x y z refc182 nx ny nz cur seg\n');
% fprintf(fileID,'%.6f %.6f %.6f %.6f %4d %.6f %.6f %.6f %.6f %d\n',ptc);
% % close all;
% fclose(fileID);

UTC = GPS_t(:,1);
xyz = GPS_t(:,2:4);
prh = GPS_t(:,[6, 7, 5]);
frame_no = GPS_t(:,8);
frame_len = length(frame_no);
index = 1:para.image_skip_num:frame_len;

if para.frame_use_num ~= -1
    index = index(1:min([para.frame_use_num, end]));
end
img_list = cell(length(index),1);
for i = 1:length(index)
    image_name = sprintf('image_%06d.jpg',frame_no(index(i)));
    img_list{i}.image = imread([dir_image_name image_name]);
    img_list{i}.frame_no = frame_no(index(i));
    img_list{i}.UTC = UTC(index(i));
    img_list{i}.xyz = xyz(index(i),:);
    img_list{i}.prh = prh(index(i),:);
end