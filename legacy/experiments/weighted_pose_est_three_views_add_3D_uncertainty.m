% First triangulate 3D points from binocular cameras and estimate their
% uncertainties, 
% and then solve a weighted PnP problem, finally apply a one-step weighted pnp GN iteration.
function [var_est,R_est,t_est,R_GN,t_GN] = weighted_pose_est_three_views_add_3D_uncertainty(z_h,y_h,x_h,R1,t1) 
m=size(y_h,2); % number of point
use_pca = 0;
if size(z_h,2)<30
    use_pca = 1;
    var_est = 1;
else
    %% noise variance estimation
    bar_A=zeros(m,9);
    bar_Y=zeros(3,3);
    H=[1 0 0;0 1 0];
    for i=1:m
        bar_A(i,:)=kron(z_h(:,i)',y_h(:,i)');
        bar_Y=bar_Y+z_h(:,i)*z_h(:,i)'/m;
    end
    Q_m=bar_A'*bar_A/m;
    S_m=kron(bar_Y,[H;0 0 0]);
    var_est=1/max(eig(Q_m\S_m));
end


%% 3D point triangulation and uncertainty estimation
% obtain coordinates in the y camera frame
pt_3d=zeros(3,m);
cov_P=zeros(3,3*m);
for i=1:m
    [pt_3d(:,i),cov_P(:,3*i-2:3*i)]=uncertain_aware_triangulation2_used(y_h(:,i),x_h(:,i),R1',-R1'*t1,var_est);
end

z_depth = pt_3d(3, :);
w = 1 ./ (z_depth.^(1.5)); % 1xN 鐨勭煩闃碉紝w 鏄? z 杞存繁搴︾殑鍊掓暟

% 鎵惧埌鐭╅樀 P 涓殑 Inf 鎴? -Inf 鍊?
inf_indices = isinf(pt_3d);

% 鎵撳嵃鍑哄寘鍚? Inf 鎴? -Inf 鐨勫厓绱犱綅缃?
[row, col] = find(inf_indices);
if ~isempty(row)  % 褰撴壘鍒癐nf鍏冪礌鏃舵墠鎵撳嵃
    disp('Inf 鎴? -Inf 鐨勪綅缃細');
    disp([row, col]);
end


%% EIV pnp pose estimation
if use_pca==1
    [R_est,t_est,~,~]=PnP_PCA(pt_3d,z_h(1:2,:));
else
%     [R_est,t_est] = EIV_PnP2(pt_3d,cov_P,z_h(1:2,:),w);
    [R_est,t_est,~] = L1_norm_PnP(pt_3d,z_h(1:2,:));
%     removal_ratio=0.06;    % important parameter to be refined!!!
%     removal_num=ceil(removal_ratio*m);
%     remain_num=m-removal_num;
%     pt_3d=pt_3d(:,index(1:remain_num));
%     z_h=z_h(:,index(1:remain_num));
%     w=w(index(1:remain_num));
%     m=size(z_h,2);
%     cov_PP=zeros(3,3*m);
%     for i=1:m
%         cov_PP(:,3*i-2:3*i)=cov_P(:,3*index(i)-2:3*index(i));
%     end
%     cov_P=cov_PP;
%         [R_est,t_est] = EIV_PnP2(pt_3d,cov_P,z_h(1:2,:),w);

end




%% weighted pnp iteration

% 优化器配置
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
                          'Algorithm', 'levenberg-marquardt');

    % 构造初始优化变量
    init_kexi = [anti_skew_symmetric(logm(R_est));t_est];

    % 优化
    opt_kexi = lsqnonlin(@myfun, init_kexi, [], [], options);
    opt_T=cal_T_from_kexi(opt_kexi);
    R_GN=opt_T(1:3,1:3);
    t_GN=opt_T(1:3,4);

    opt_cost=myfun(opt_kexi);



    function g = myfun(kexi)
        T=cal_T_from_kexi(kexi);
%         delta=0.5*10^(-4);
        delta=2*10^(-2);
        X=pt_3d;
        x=z_h(1:2,:);
        E=[1 0 0;0 1 0];
        e3 = [0;0;1];
        g=E* (T(1:3,1:3)*X+T(1:3,4));
        h=e3'* (T(1:3,1:3)*X+T(1:3,4));
        f=g./h;
        ww=kron(w',[1;1]);
%         d=ww.*(x(:) - f(:));
        d=x(:) - f(:);
        g=truncate_loss(d,delta);
    end
end

function T = cal_T_from_kexi(kexi)
    T = [expm(skew_symmetric(kexi(1:3))) kexi(4:6); 0,0,0,1];
end

function L = truncate_loss(e, delta)
    % 截断核函数
    % 输入:
    %   e: 误差值 (可以是标量或向量)
    %   delta: 阈值参数 (标量)
    % 输出:
    %   L: 截断损失值 (与输入 e 的维度相同)

    % 计算误差的绝对值
    abs_e = abs(e);
    
    % 应用截断
    L = min(abs_e, delta);
    
    % 为了保持数值稳定性，对误差进行归一化
    L = L / delta;
end
