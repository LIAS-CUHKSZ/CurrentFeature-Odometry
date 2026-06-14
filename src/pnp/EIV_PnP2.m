% solve weighted PnP problem and obtain a consistent estimate.

function [R_est,t_est] = EIV_PnP2(P,cov_P,z,w)  %P is 3×n, cov_P is 3×3n, z is 2×n, w is 1×n

n=size(P,2);
A=zeros(2*n,11);
b=zeros(2*n,1);
bar_P=sum(P,2)/n;
for i=1:n
    A(2*i-1,:)=[-w(i)*z(1,i)*(P(:,i)-bar_P)' w(i)*P(:,i)' w(i) zeros(1,4)];
    A(2*i,:)=[-w(i)*z(2,i)*(P(:,i)-bar_P)' zeros(1,4) w(i)*P(:,i)' w(i)];
    b(2*i-1:2*i)=w(i)*z(:,i);
end
ATA=A'*A/n;
ATb=A'*b/n;

delta_ATA=zeros(11,11);
for i=1:n
    delta_ATA(1:3,1:3)=delta_ATA(1:3,1:3)+w(i)^2*norm(z(:,i))^2*cov_P(:,3*i-2:3*i)/2;
    delta_ATA(1:3,4:6)=delta_ATA(1:3,4:6)-w(i)^2*z(1,i)*cov_P(:,3*i-2:3*i);
    delta_ATA(1:3,8:10)=delta_ATA(1:3,8:10)-w(i)^2*z(2,i)*cov_P(:,3*i-2:3*i);
    delta_ATA(4:6,4:6)=delta_ATA(4:6,4:6)+w(i)^2*cov_P(:,3*i-2:3*i)/2;
    delta_ATA(8:10,8:10)=delta_ATA(8:10,8:10)+w(i)^2*cov_P(:,3*i-2:3*i)/2;
end
delta_ATA=delta_ATA/n;
delta_ATA=delta_ATA+delta_ATA';

est_bias_eli=(ATA-delta_ATA)\ATb;
ratio = (trace(delta_ATA) / trace(ATA)) * 100;


% if abs(ratio) > 0.5
%     fprintf(' %.2f%% ', ...
%         ratio);
% end

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