QQ=eye(5);
%QQ(3,3)=20;
RR=eye(2);
K=lqrd(s3.a, s3.b, QQ, RR, Ts);