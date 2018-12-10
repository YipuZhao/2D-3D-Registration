function [point_voxel_arr] = pointCloudVoxelize(reg_points, bin_point_num)
%%
point_voxel_arr = [1:length(reg_points); zeros(1, length(reg_points))];
pts = reg_points(:, 2:4);
OT = OcTree(pts, 'binCapacity', bin_point_num);

figure
boxH = OT.plot;
cols = lines(OT.BinCount);
doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
for i = 1:OT.BinCount
    set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i))
    doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:))
end
axis image, view(3)

voxel_idx_mapping = unique(OT.PointBins);
voxel_idx_mapping = [voxel_idx_mapping, (1:length(voxel_idx_mapping))'];

for i=min(OT.PointBins) : max(OT.PointBins)
    point_idx = find(OT.PointBins == i);
    val_idx = find(voxel_idx_mapping(:, 1) == i);
    point_voxel_arr(2, point_idx') = voxel_idx_mapping(val_idx, 2);
end

% point_voxel_arr(:, 2) = OT.PointBins;