function [xk, Pk] = updateKF(sys, xk, Pk, yk, Rk)

A = sys.A;
B = sys.B;
C = sys.C;
Q = sys.Q; 
I = eye(size(A));

Kk = Pk*C'*inv(C*Pk*C' + Rk); % kalman gain
xk =  xk + Kk*(yk - C*xk); % updated state
Pk = (I - Kk*C)*Pk; % updated state covariance

end