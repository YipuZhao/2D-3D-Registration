function [points, img_list] = loadData(dir_name, dir_image_name, para)

% GPS_short
%UTC x y z h p r frameno
GPS_t = textread([dir_name para.data_name '_' para.GPS_name], '', 'delimiter', ' ', 'headerlines',2);

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
    image_name = sprintf(para.image_name_format,frame_no(index(i)));
    img_list{i}.image = imread([dir_image_name image_name]);
    img_list{i}.frame_no = frame_no(index(i));
    img_list{i}.UTC = UTC(index(i));
    img_list{i}.xyz = xyz(index(i),:);
    img_list{i}.prh = prh(index(i),:);
    % check if cluster label file exist
    %     img_list{i}.fname = [dir_image_name sprintf(para.label_name_format,frame_no(index(i)))];
    %     fid = fopen(img_list{i}.fname, 'r');
    %     if fid ~= -1
    %         img_list{i}.label = fread(fid, 1, 'uint8');
    %         fclose (fid);
    %     else
    %         img_list{i}.label = -1;
    %     end
end

% point cloud
% if strcmp(para.camera_type, 'kitti')
%     points = [];
%     for i = 1:length(index)
%         fid = fopen(sprintf(para.point_cloud_format,para.dir_velo_name,frame_no(index(i))),'rb');
%         velo = fread(fid,[9 inf],'single')';
%         fclose(fid);
%         points = [points; [frame_no(index(i))*ones(size(velo,1),1), velo]];
%     end
% else
%     % UTC x y z refc182 nx ny nz cur edge
%     points = textread([dir_name para.data_name '_' para.point_cloud_name], '', 'delimiter', ' ', 'headerlines',2);
%     points(:,5) = double(points(:,5))/max(points(:,5));
% %     if para.pavement_cut==1
% %         points = points((points(:,10)~=0),:);
% %     elseif para.pavement_cut==2
% %         points = points((points(:,10)==0),:);
% %     end
% end

% UTC x y z refc182 refc_eq nx ny nz curvature gradm_ref gradm_rng
if strcmp(para.camera_type, 'kitti')
    fileID = fopen([dir_name para.data_name '_' para.point_cloud_name]);
    % load points number
    formatSpec = '%d';
    N = 1;
    points_num = textscan(fileID,formatSpec,N,'Delimiter',' ');
    % load points properties
    formatSpec = '%s';
    N = 12;
    points_prop = textscan(fileID,formatSpec,N,'Delimiter',' ');
    points_cell = textscan(fileID,'%f %f %f %f %f %f %f %f %f %f %f %f');
    fclose(fileID);
    points = cell2mat(points_cell);
    %     points = textread([dir_name para.data_name '_' para.point_cloud_name], '', 'delimiter', ' ', 'headerlines',2);
else
    points_h = textread([dir_name para.data_name '_' para.point_cloud_name], '', 'delimiter', ' ', 'headerlines',2);
    points_v = textread([dir_name para.data_name '_v_' para.point_cloud_name], '', 'delimiter', ' ', 'headerlines',2);
    points = [points_h; points_v];
end
% points(:,5) = double(points(:,5))/max(points(:,5));