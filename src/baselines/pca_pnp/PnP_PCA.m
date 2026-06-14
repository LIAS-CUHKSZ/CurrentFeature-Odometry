function [R_PCA,t_PCA,R_PCA_iter,t_PCA_iter]=PnP_PCA(P_3D,p_2d) % P_3D is 3xn, p_2d is 2xn

n=size(P_3D,2);
L=[1 -1 -1 0 0 0 0 0 0 1;
    0 0 0 2 0 0 0 0 -2 0;
    0 0 0 0 2 0 0 2 0 0;
    0 0 0 2 0 0 0 0 2 0;
    -1 1 -1 0 0 0 0 0 0 1;
    0 0 0 0 0 2 -2 0 0 0;
    0 0 0 0 2 0 0 -2 0 0;
    0 0 0 0 0 2 2 0 0 0;
    -1 -1 1 0 0 0 0 0 0 1];
A=zeros(2*n,9);
B=zeros(2*n,3);
for i=1:n
    A(2*i-1:2*i,:)=[P_3D(:,i)' zeros(1,3) -p_2d(1,i)*P_3D(:,i)';zeros(1,3) P_3D(:,i)' -p_2d(2,i)*P_3D(:,i)'];
    B(2*i-1:2*i,:)=[1 0 -p_2d(1,i);0 1 -p_2d(2,i)];
end
C=-inv(B'*B)*B'*A;
D=A+B*C;
E=D*L;
Q=E'*E;
[V,Lambda]=eig(Q);  % use the last three columns of V and three elements of Lambda (three largest eigenvalues) to obtain initial PCA-based pose estimate

[s1, s2, s3]=solve_three_quadrics_with_Q(sqrt(Lambda(10,10))*V(:,10),sqrt(Lambda(9,9))*V(:,9),sqrt(Lambda(8,8))*V(:,8),Q);
s=[s1,s2,s3]';
S=[s1^2 s2^2 s3^2 s1*s2 s1*s3 s2*s3 s1 s2 s3 1]';
phi=L*S;
bar_R=[phi(1:3)';phi(4:6)';phi(7:9)'];
R_PCA=bar_R/(1+s'*s);
tao=C*phi;
t_PCA=tao/(1+s'*s);

%% iterative refinement
hat_A=zeros(2*n,9);
hat_B=zeros(2*n,3);
for i=1:n
    di=bar_R(3,:)*P_3D(:,i)+tao(3);
    hat_A(2*i-1:2*i,:)=A(2*i-1:2*i,:)/di;
    hat_B(2*i-1:2*i,:)=B(2*i-1:2*i,:)/di;
end
hat_C=-inv(hat_B'*hat_B)*hat_B'*hat_A;
hat_D=hat_A+hat_B*hat_C;
hat_Q=L'*hat_D'*hat_D*L;
partial_S_s=[2*s(1) 0 0 s(2) s(3) 0 1 0 0 0;0 2*s(2) 0 s(1) 0 s(3) 0 1 0 0;0 0 2*s(3) 0 s(1) s(2) 0 0 1 0];
kapa=partial_S_s*2*hat_Q*S;
partial_S_s1s=[2 zeros(1,9);zeros(1,3) 1 zeros(1,6);zeros(1,4) 1 zeros(1,5)];
partial_S_s2s=[zeros(1,3) 1 zeros(1,6);0 2 zeros(1,8);zeros(1,5) 1 zeros(1,4)];
partial_S_s3s=[zeros(1,4) 1 zeros(1,5);zeros(1,5) 1 zeros(1,4);zeros(1,2) 2 zeros(1,7)];
J=[partial_S_s*2*hat_Q*partial_S_s(1,:)'+partial_S_s1s*2*hat_Q*S partial_S_s*2*hat_Q*partial_S_s(2,:)'+partial_S_s2s*2*hat_Q*S partial_S_s*2*hat_Q*partial_S_s(3,:)'+partial_S_s3s*2*hat_Q*S];
delta=inv(J)*kapa;

syms lambda
kapa_lambda=kapa_fun(s-lambda*delta,hat_Q);
g_lambda=kapa_lambda(1)^2+kapa_lambda(2)^2+kapa_lambda(3)^2;
dg_lambda=diff(g_lambda,lambda);
coeff=sym2poly(dg_lambda);
result = roots(coeff);
m=size(result);
index=[];
for i=1:m
    if ~isreal(result(i))
        index=[index i];
    end
end
result(index)=[];
opt_lambda=result(1);
opt_obj=kapa_fun(s-opt_lambda*delta,hat_Q)'*kapa_fun(s-opt_lambda*delta,hat_Q);
m=size(result,1);
for i=1:m
    temp_obj=kapa_fun(s-result(i)*delta,hat_Q)'*kapa_fun(s-result(i)*delta,hat_Q);
    if temp_obj<opt_obj
        opt_obj=temp_obj;
        opt_lambda=result(i);
    end
end
s=s-opt_lambda*delta;
s1=s(1);
s2=s(2);
s3=s(3);
S=[s1^2 s2^2 s3^2 s1*s2 s1*s3 s2*s3 s1 s2 s3 1]';
phi=L*S;
bar_R=[phi(1:3)';phi(4:6)';phi(7:9)'];
R_PCA_iter=bar_R/(1+s'*s);
tao=hat_C*phi;
t_PCA_iter=tao/(1+s'*s);




