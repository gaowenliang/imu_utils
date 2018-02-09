function C = fit_allan( tau, sig, M)
    X=tau';
    Y=sig';
    B=zeros(1,2*M+1);
    F=zeros(length(X),2*M+1);

for i=1:2*M+1
    kk=i-M-1;
    F(:,i)=X.^kk;
end

A=F'*F;
B=F'*Y;
C=A\B;
