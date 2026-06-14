function kapa_s=kapa_fun(s,Q)
s1=s(1);
s2=s(2);
s3=s(3);
S=[s1^2; s2^2; s3^2; s1*s2; s1*s3; s2*s3; s1; s2; s3; 1];
partial_S_s=[2*s1 0 0 s2 s3 0 1 0 0 0;0 2*s2 0 s1 0 s3 0 1 0 0;0 0 2*s3 0 s1 s2 0 0 1 0];
kapa_s=partial_S_s*2*Q*S;
