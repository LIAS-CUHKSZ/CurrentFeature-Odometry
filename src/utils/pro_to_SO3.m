function R=pro_to_SO3(R_est)

[U,~,V] = svd(R_est);
R=U*diag([1 1 det(U*V')]')*V';