function [tmp_x0, t, R, inv_R, err_val] = addError(err_type, tmp_x0, iter_no, step_num)

% rad
d_heading = deg2rad(0.1);
d_pitching = deg2rad(0.1);
d_rolling = deg2rad(0.1);
% meter
d_x = 0.025;
d_y = 0.025;
d_z = 0.025;

err_x0 = zeros(1, 6);

switch err_type
    case 'heading'
        err_x0(6) = d_heading * (iter_no-step_num-1);
        err_val = err_x0(6);
    case 'rolling'
        err_x0(5) = d_rolling * (iter_no-step_num-1);
        err_val = err_x0(5);
    case 'pitching'
        err_x0(4) = d_pitching * (iter_no-step_num-1);
        err_val = err_x0(4);
    case 'shift_x'
        err_x0(1) = d_x * (iter_no-step_num-1);
        err_val = err_x0(1);
    case 'shift_y'
        err_x0(2) = d_y * (iter_no-step_num-1);
        err_val = err_x0(2);
    case 'shift_z'
        err_x0(3) = d_z * (iter_no-step_num-1);
        err_val = err_x0(3);
end

tmp_x0 = tmp_x0 + err_x0;

R = makeRotationMatrix(tmp_x0(4:6));
inv_R = inv(R);
t = tmp_x0(1:3)';