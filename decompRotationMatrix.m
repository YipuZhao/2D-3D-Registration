function [angle_vec] = decompRotationMatrix(rotate_mat)
%%
% angle_vec: heading, pitch, roll
% rot_z = [cos(angle_vec(3)), -sin(angle_vec(3)), 0;
%          sin(angle_vec(3)), cos(angle_vec(3)), 0;
%          0, 0, 1];
%
% rot_x = [1, 0, 0;
%          0, cos(angle_vec(1)), -sin(angle_vec(1));
%          0, sin(angle_vec(1)), cos(angle_vec(1))];
%
% rot_y = [cos(angle_vec(2)), 0, sin(angle_vec(2));
%          0, 1, 0;
%          -sin(angle_vec(2)), 0, cos(angle_vec(2))];


% rotate_mat = rot_z * rot_x * rot_y;

if rotate_mat(3,2) < 1
    %
    if rotate_mat(3,2) > -1
        theta_x = asin(rotate_mat(3,2));
        theta_y = atan2(-rotate_mat(3,1), rotate_mat(3,3));
        theta_z = atan2(-rotate_mat(1,2), rotate_mat(2,2));
    else
        theta_x = -pi/2;
        theta_y = 0;
        theta_z = -atan2(rotate_mat(1,3), rotate_mat(1,1));
    end
else
    theta_x = pi/2;
    theta_y = 0;
    theta_z = atan2(rotate_mat(1,3), rotate_mat(1,1));
end

angle_vec = [theta_x, theta_y, theta_z];

