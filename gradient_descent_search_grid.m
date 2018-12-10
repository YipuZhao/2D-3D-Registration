function [xk] = gradient_descent_search_grid (x0_hl, img_list, points, para)
MAX_ITER = 300;
DTOR = 1/180*pi;
gama_trans_u = 0.1;
gama_trans_l = 0.001;
gama_rot_u = 1*DTOR;
gama_rot_l = 0.01*DTOR;

deltaPara = [0.01, 0.01, 0.01, 0.1*DTOR, 0.1*DTOR, 0.1*DTOR];

xk  = x0_hl;
xk_minus_1 = x0_hl;

delF_delPara_ = [0, 0, 0, 0, 0, 0];

[mi_cost_core, point_num_core, point_area_core] = mi_cost_2d(xk, img_list, points, para, '');
point_num_valid = point_num_core>para.GRID_THRD & point_area_core>para.AREA_THRD;

skip_step = para.do_registration;
if skip_step>0
    gama_rot_u = gama_rot_u/(1.2^skip_step);
    gama_rot_l = gama_rot_l/(1.2^skip_step);
    gama_trans_u = gama_trans_u/(1.2^skip_step);
    gama_trans_l = gama_trans_l/(1.2^skip_step);
    deltaPara = deltaPara/(1.1^skip_step);
    step = skip_step;
else
    step = 0;
end

core_bias_multi = [-2, -1, 1, 2]; % f(x-2dx), f(x-dx), f(x+dx), f(x+2dx)
for index = 1 : MAX_ITER
    % get [x-2dx; x-dx; x+dx; x+2dx] for gradient computation
    delta = ones(6*length(core_bias_multi),1)*xk;
    core_bias = core_bias_multi'*deltaPara;
    bias = zeros(size(delta));
    for k = 1:6
        bias((1:length(core_bias_multi))+(k-1)*length(core_bias_multi),k) = core_bias(:,k);
    end
    delta = delta + bias;
    
    % compute v = f(x)
    mi_cost = zeros(size(delta,1), length(mi_cost_core));
    point_num = zeros(size(delta,1), length(mi_cost_core));
    point_area = zeros(size(delta,1), length(mi_cost_core));
    parfor s = 1:size(delta,1)
        [mi_cost(s,:), point_num(s,:), point_area(s,:)] = mi_cost_2d(delta(s,:), img_list, points, para,'');
    end
    v = reshape(poseAssess(mi_cost, point_num, point_num_valid, para.GRID_THRD),length(core_bias_multi),[])';
    % gradient for x, y, z, p, r, h
    delF_delPara = (v(:,1) - 8*v(:,2) + 8*v(:,3) - v(:,4))'./(12*deltaPara);
    norm_delF_del_trans = sqrt(sum(delF_delPara(1:3).^2));
    norm_delF_del_rot = sqrt(sum(delF_delPara(4:6).^2));
    delF_delPara(1:3) = delF_delPara(1:3)/norm_delF_del_trans;
    delF_delPara(4:6) = delF_delPara(4:6)/norm_delF_del_rot;
    % print status
    disp(sprintf('step %d',index));
    disp(sprintf('k = %d, df = %0.6f, f = (%0.6f, %0.6f, %0.6f, %0.6f)\n', [(1:6)', delF_delPara', v]'));
    
    % step length
    delta_trans = sum((xk(1:3)-xk_minus_1(1:3)).^2);
    delta_rot = sum((xk(4:6)-xk_minus_1(4:6)).^2);
    
    %get the scaling factor
    if (delta_trans > 0)
        temp_deno_trans = abs(sum((xk(1:3)-xk_minus_1(1:3)).*(delF_delPara(1:3) - delF_delPara_(1:3))));
        gama_trans = delta_trans/temp_deno_trans;
    else
        gama_trans = gama_trans_u;
    end
    if (delta_rot > 0)
        temp_deno_rot = abs(sum((xk(4:6)-xk_minus_1(4:6)).*(delF_delPara(4:6) - delF_delPara_(4:6))));
        gama_rot = delta_rot/temp_deno_rot;
    else
        gama_rot = gama_rot_u;
    end
    if (gama_trans > gama_trans_u)
        gama_trans = gama_trans_u;
    end
    if (gama_trans < gama_trans_l)
        gama_trans = gama_trans_l;
    end
    if (gama_rot > gama_rot_u)
        gama_rot = gama_rot_u;
    end
    if (gama_rot < gama_rot_l)
        gama_rot = gama_rot_l;
    end
    
    % next step
    xk_minus_1 = xk;
    xk(1:3)= xk(1:3) + gama_trans*delF_delPara(1:3);
    xk(4:6) = xk(4:6) + gama_rot*delF_delPara(4:6);
    
    % check whether improved
    [mi_cost_curr, point_num_curr] = mi_cost_2d(xk, img_list, points, para,'');
    v2 = poseAssess([mi_cost_core; mi_cost_curr], [point_num_core; point_num_curr], point_num_valid, para.GRID_THRD);
    f_core = v2(1);
    f_curr = v2(2);
    
    % print status
    disp(sprintf('core f = %0.6f, (%0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f)', f_core,xk_minus_1));
    disp(sprintf('curr f = %0.6f, (%0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f)', f_curr,xk));
    disp(sprintf('change (%0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f)', xk-xk_minus_1));
    if (f_curr < f_core) % worse, get back
        xk = xk_minus_1;
        disp(sprintf ('next=, step =  %d\n',step));
        % change step length
        gama_rot_u = gama_rot_u/1.2;
        gama_rot_l = gama_rot_l/1.2;
        gama_trans_u = gama_trans_u/1.2;
        gama_trans_l = gama_trans_l/1.2;
        deltaPara = deltaPara/1.1;
        
        step = step+1;
        if (deltaPara(1) < 0.001)
            break;
        end
    else % improved, change core
        mi_cost_core = mi_cost_curr;
        point_num_core = point_num_curr;
        disp(sprintf ('next-, step = %d\n\n', step));
        
        delF_delPara_ = delF_delPara;
    end
end