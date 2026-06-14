function [s1, s2, s3]=solve_three_quadrics_with_Q(v1,v2,v3,Q)

syms S1 S2 S3
E1= v1'*[S1^2 S2^2 S3^2 S1*S2 S1*S3 S2*S3 S1 S2 S3 1]'==0;
E2= v2'*[S1^2 S2^2 S3^2 S1*S2 S1*S3 S2*S3 S1 S2 S3 1]'==0;
E3= v3'*[S1^2 S2^2 S3^2 S1*S2 S1*S3 S2*S3 S1 S2 S3 1]'==0;
result = solve(E1,E2,E3);

s1_temp=vpa(result.S1);
s2_temp=vpa(result.S2);
s3_temp=vpa(result.S3);

s1_temp=eval(s1_temp);
s2_temp=eval(s2_temp);
s3_temp=eval(s3_temp);

m=size(s1_temp,1);
index=[];
for i=1:m
    if ~isreal(s1_temp(i)) || ~isreal(s2_temp(i)) || ~isreal(s3_temp(i))
        index=[index i];
    end
end
s1_temp(index)=[];
s2_temp(index)=[];
s3_temp(index)=[];

m=size(s1_temp,1);
obj_value=zeros(m,1);
for i=1:m
    S=[s1_temp(i)^2 s2_temp(i)^2 s3_temp(i)^2 s1_temp(i)*s2_temp(i) s1_temp(i)*s3_temp(i) s2_temp(i)*s3_temp(i) s1_temp(i) s2_temp(i) s3_temp(i) 1]';
    obj_value(i)=S'*Q*S;
end

[~,index]=min(obj_value);

s1=s1_temp(index);
s2=s2_temp(index);
s3=s3_temp(index);

if m==0
    s1=0;
    s2=0;
    s3=0;
end