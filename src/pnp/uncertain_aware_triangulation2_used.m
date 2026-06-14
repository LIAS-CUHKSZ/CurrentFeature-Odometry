% R and t is the pose that y camera with respect to x camera
% P is the coordinates in y camera frame
% estimate P using (A'*A)\(A'*b), and assume both y_h and x_h contain noise

function [P,cov] = uncertain_aware_triangulation2_used(y_h,x_h,R,t,var)

%% triangulation
f=y_h(3);
y=y_h(1:2);
x=x_h(1:2);
A=[skew_symmetric(y_h);skew_symmetric(x_h)*R];
b=[zeros(3,1);-cross(x_h,t)];

P=(A'*A)\(A'*b);

%% uncertainty estimation
M1=[0 0 0;0 0 -1;0 1 0];
M2=[0 0 1;0 0 0;-1 0 0];
partial_ATA_x1=-R'*M1*skew_symmetric(x_h)*R-R'*skew_symmetric(x_h)*M1*R;
partial_ATA_x2=-R'*M2*skew_symmetric(x_h)*R-R'*skew_symmetric(x_h)*M2*R;
partial_ATb_x1=R'*M1*skew_symmetric(x_h)*t+R'*skew_symmetric(x_h)*M1*t;
partial_ATb_x2=R'*M2*skew_symmetric(x_h)*t+R'*skew_symmetric(x_h)*M2*t;

partial_ATA_y1=-M1*skew_symmetric(y_h)-skew_symmetric(y_h)*M1;
partial_ATA_y2=-M2*skew_symmetric(y_h)-skew_symmetric(y_h)*M2;

partial_P_x=-inv(A'*A)*[partial_ATA_x1*P partial_ATA_x2*P]+inv(A'*A)*[partial_ATb_x1 partial_ATb_x2];
partial_P_y=-inv(A'*A)*[partial_ATA_y1*P partial_ATA_y2*P];

cov=partial_P_x*diag([var var])*partial_P_x'+partial_P_y*diag([var var])*partial_P_y';
