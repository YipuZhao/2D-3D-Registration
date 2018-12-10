for img_idx = 1:10%frame_num
    prior_image = img_list{img_idx};
    [~, t, R, inv_R, ~] = addError(para.err_list{1}, para.x0, 1+step_num, step_num);
    [u, v, points_id] = imageProjection_upd(points, prior_image, para, t, R, inv_R);
    [u, v, points_id] = occludedPointsRemoval(u, v, points_id, prior_image, points);
    % project rgb info from image to point cloud
    prior_int_img = rgb2gray(prior_image.image);
    prior_inteq_img = adapthisteq(prior_int_img);
    %         tmp_img = shadowremoval(im2double(prior_image.image), 'ycbcr');
    %         prior_y_img = tmp_img(:,:,1);
    %         tmp_img = rgb2hsv(double(prior_image.image));
    %         prior_h_img = tmp_img(:,:,1);
    %
    %         prior_int_arr = im2double(prior_int_img);
    prior_inteq_arr = im2double(prior_inteq_img);
    %         prior_y_arr = im2double(prior_y_img);
    %         prior_h_arr = im2double(prior_h_img);
    %
    %         prior_int_arr = prior_int_arr(:)';
    %         points(points_id, 13) = prior_int_arr(v + (u-1)*para.cam.height)';
    %         prior_inteq_arr = prior_inteq_arr(:)';
    %         points(points_id, 14) = prior_inteq_arr(v + (u-1)*para.cam.height)';
    %         prior_y_arr = prior_y_arr(:)';
    %         points(points_id, 15) = prior_y_arr(v + (u-1)*para.cam.height)';
    %         prior_h_arr = prior_h_arr(:)';
    %         points(points_id, 16) = prior_h_arr(v + (u-1)*para.cam.height)';
    prior_inteq_arr = prior_inteq_arr(:)';
    prior_inteq_pt = prior_inteq_arr(v + (u-1)*para.cam.height)';
    
    for pt_idx=1:length(points_id)
        color_buf{points_id(pt_idx)} = [color_buf{points_id(pt_idx)}; img_idx, prior_inteq_pt(pt_idx)];
    end
end