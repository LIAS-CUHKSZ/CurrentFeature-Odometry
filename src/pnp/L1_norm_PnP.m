% solve error-in-variable PnP problem and obtain a consistent estimate. If
% the 2D points are noise-free, one can set var=0.

function [R_est,t_est,index] = L1_norm_PnP(P,z)  %P is 3×n, z is 2×n

n=size(P,2);
A=zeros(2*n,11);
b=zeros(2*n,1);
bar_P=sum(P,2)/n;
for i=1:n
    A(2*i-1,:)=[-z(1,i)*(P(:,i)-bar_P)' P(:,i)' 1 zeros(1,4)];
    A(2*i,:)=[-z(2,i)*(P(:,i)-bar_P)' zeros(1,4) P(:,i)' 1];
    b(2*i-1:2*i)=z(:,i);
end

%% L1_norm optimization
est_bias_eli=L1_norm_opt_lp(A,b);

%% pose recovery
bias_eli_rotation=[est_bias_eli(4:6)';est_bias_eli(8:10)';est_bias_eli(1:3)'];
bias_eli_t=[est_bias_eli(7) est_bias_eli(11) 1-est_bias_eli(1:3)'*bar_P]';
sign_corr=sign(det(bias_eli_rotation));
bias_eli_rotation=bias_eli_rotation*sign_corr;
bias_eli_t=bias_eli_t*sign_corr;
normalize_factor=(det(bias_eli_rotation))^(1/3);
bias_eli_rotation=bias_eli_rotation/normalize_factor; % initial rotation estimate
t_est=bias_eli_t/normalize_factor; % initial translation estimate
[U,~,V] = svd(bias_eli_rotation);
R_est=U*diag([1 1 det(U*V')]')*V';

%% reprojection error sorting
repro_err=zeros(n,1);
P_z_est=R_est*P+t_est;
for i=1:n
    zi_repro=P_z_est(1:2,i)/P_z_est(3,i);
    repro_err(i)=norm(z(:,i)-zi_repro);
end
[~,index]=sort(repro_err);