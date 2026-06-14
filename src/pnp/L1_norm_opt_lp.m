% solve L1-norm linear least squares using linear programming

function x=L1_norm_opt_lp(A,b)

n=size(A,2);  % number of variables
m=size(A,1);  % number of equations

bar_A1=zeros(m,n+m);
bar_A2=zeros(m,n+m);
c=[zeros(n,1);ones(m,1)];
for i=1:m
    bar_A1(i,1:n)=A(i,:);
    bar_A1(i,n+i)=-1;
    bar_A2(i,1:n)=-A(i,:);
    bar_A2(i,n+i)=-1;
end
bar_A=[bar_A1;bar_A2];
bar_b=[b;-b];

options = optimoptions('linprog','Algorithm','interior-point','Display','none');


x = linprog(c,bar_A,bar_b,[],[],[],[],options);