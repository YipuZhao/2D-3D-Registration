% file: shadowremoval.m
%   Single shadow removal from colour images (with 4 different algorithms).
%
% Output: Image with the shadows remover from it.
%
% Parameters:
%   - image:        Coulour image containig the shadow to be removed.
%                   Image is in RGB space, with colour values between
%                   0 and 1. The image contains one homogenious texture,
%                   in part in the shadowand in part in the light.
%   - type:         string defining the type of shadow removal method to be
%                   applied:
%                   - 'additive': balancing the pixel intensities in the
%                     shadow and in the light by addition of the difference
%                     of pixel averages to the shadow pixels.
%                   - 'basiclightmodel': Simple crisp shadow removal derved
%                     from the light model containing a global, and a
%                     directed light.
%                   - 'advancedlightmodel': Advanced shadow removal based
%                     vased on the same light model containing two types
%                     of light.
%                     Needs a 'fuzzy' shadowmap containing for each pixel,
%                     the level of 'litness' to give correst results,
%                     unless it is the same a 'basiclightmodel'.
%                   - 'ycbcr': combination of the additive and light model
%                     based methods in ycbcr colourspace.
%   - mask:         - The mask describing the pixel litness on the image.
%                     (Segmentation of the shadow.)
%                     Pixels in the shadow are assigned with the walue of
%                     1, pixel in the light are marked with a 0, and a
%                     smooth transition is allowed.
%                     (Optional) If not given a global thresholding will be
%                     used for shadow detection.

function result = shadowremoval(image, type, mask)

    % computing size of the image
    s_im = size(image);
    
    % creating a shadow segmentation if no mask is available
    if (~exist('mask','var'))
        gray = rgb2gray(image);
        mask = 1-double(im2bw(gray, graythresh(gray)));
    end

    % structuring element for the shadow mask buring, and the shadow/light
    % core detection
    strel = [0 1 1 1 0; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 0 1 1 1 0];
    
    % computing shadow/light  core (pixels not on the blured adge of the
    % shadow area)
    shadow_core = imerode(mask, strel);
    lit_core = imerode(1-mask, strel);
    
    % smoothing the mask
    smoothmask = conv2(mask, strel/21, 'same');
    
    % averaging pixel intensities in the shadow/lit areas
    shadowavg_red = sum(sum(image(:,:,1).*shadow_core)) / sum(sum(shadow_core));
    shadowavg_green = sum(sum(image(:,:,2).*shadow_core)) / sum(sum(shadow_core));
    shadowavg_blue = sum(sum(image(:,:,3).*shadow_core)) / sum(sum(shadow_core));

    litavg_red = sum(sum(image(:,:,1).*lit_core)) / sum(sum(lit_core));
    litavg_green = sum(sum(image(:,:,2).*lit_core)) / sum(sum(lit_core));
    litavg_blue = sum(sum(image(:,:,3).*lit_core)) / sum(sum(lit_core));
    
    result = image;
    
    % implementation of the different shadow removals
    
    if strcmp(type, 'additive')
        % additive shadow removal
        
        % compiting colour difference between the shadow/lit areas
        diff_red = litavg_red - shadowavg_red;
        diff_green = litavg_green - shadowavg_green;
        diff_blue = litavg_blue - shadowavg_blue;
        
        % adding the difference to the shadow pixels
        result(:,:,1) = image(:,:,1) + smoothmask * diff_red;
        result(:,:,2) = image(:,:,2) + smoothmask * diff_green;
        result(:,:,3) = image(:,:,3) + smoothmask * diff_blue;
        
    elseif strcmp(type, 'basiclightmodel')
        % basic, light model based shadow removal
        
        % computing ratio of shadow/lit area luminance
        ratio_red = litavg_red/shadowavg_red;
        ratio_green = litavg_green/shadowavg_green;
        ratio_blue = litavg_blue/shadowavg_blue;
        
        % multipliing the shadow pixels with the raio for the correction
        result(:,:,1) = image(:,:,1).*(1-mask) + mask.*ratio_red.*image(:,:,1);
        result(:,:,2) = image(:,:,2).*(1-mask) + mask.*ratio_green.*image(:,:,2);
        result(:,:,3) = image(:,:,3).*(1-mask) + mask.*ratio_blue.*image(:,:,3);
        
    elseif strcmp(type, 'advancedlightmodel')
        % advanced, light model based shadow removal
        
        % computing ratio of the luminances of the directed, and global
        % lights
        ratio_red = litavg_red/shadowavg_red - 1;
        ratio_green = litavg_green/shadowavg_green - 1;
        ratio_blue = litavg_blue/shadowavg_blue - 1;
        
        % appliing shadow removal formula
        % (too long for the comment -> see documentation :) )
        result(:,:,1) = (ratio_red + 1)./((1-smoothmask)*ratio_red + 1).*image(:,:,1);
        result(:,:,2) = (ratio_green + 1)./((1-smoothmask)*ratio_green + 1).*image(:,:,2);
        result(:,:,3) = (ratio_blue + 1)./((1-smoothmask)*ratio_blue + 1).*image(:,:,3);

    elseif strcmp(type, 'ycbcr')
        % combined additive and light model based shadow removal in
        % ycbcr colourspace
        
        % conversion to ycbcr
        ycbcr = rgb2ycbcr(image);
        
        % computing averade channel values in ycbcr space
        shadowavg_y = sum(sum(ycbcr(:,:,1).*shadow_core)) / sum(sum(shadow_core));
        shadowavg_cb = sum(sum(ycbcr(:,:,2).*shadow_core)) / sum(sum(shadow_core));
        shadowavg_cr = sum(sum(ycbcr(:,:,3).*shadow_core)) / sum(sum(shadow_core));

        litavg_y = sum(sum(ycbcr(:,:,1).*lit_core)) / sum(sum(lit_core));
        litavg_cb = sum(sum(ycbcr(:,:,2).*lit_core)) / sum(sum(lit_core));
        litavg_cr = sum(sum(ycbcr(:,:,3).*lit_core)) / sum(sum(lit_core));
        
        % computing ratio, and difference in ycbcr space
        diff_y = litavg_y - shadowavg_y;
        diff_cb = litavg_cb - shadowavg_cb;
        diff_cr = litavg_cr - shadowavg_cr;

        ratio_y = litavg_y/shadowavg_y;
        ratio_cb = litavg_cb/shadowavg_cb;
        ratio_cr = litavg_cr/shadowavg_cr;

        % shadow correction, see formulas above
        % y channel has an additive correction
        % cb, and cr channels gets a model based correction
        res_ycbcr = ycbcr;
        res_ycbcr(:,:,1) = ycbcr(:,:,1) + mask * diff_y;
        res_ycbcr(:,:,2) = ycbcr(:,:,2).*(1-mask) + mask.*ratio_cb.*ycbcr(:,:,2);
        res_ycbcr(:,:,3) = ycbcr(:,:,3).*(1-mask) + mask.*ratio_cr.*ycbcr(:,:,3);
        
        % conversion back to rgb colourspace
        result = ycbcr2rgb(res_ycbcr);
    end
    
    
end