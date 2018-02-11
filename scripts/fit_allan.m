function C = fit_allan( tau, sig, M)
    X=tau';
    Y=sig';
    B=zeros(1,2*M+1);
    F=zeros(length(X),2*M+1);
%     disp (X)
%     disp (Y)

for i=1:2*M+1
    kk=i-M-1;
    F(:,i)=X.^kk;
end
%     disp (F)

A=F'*F;
%     disp (A)

B=F'*Y;
%     disp (B)

C=A\B;
%     disp (C)
