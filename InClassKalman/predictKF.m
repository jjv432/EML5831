function  [xk, Pk] = predictKF(sys, xk, Uk, Pk)

A = sys.A;
B = sys.B;
C = sys.C;
Q = sys.Q; 

xk = A*xk + B*Uk;
Pk = A*Pk*A' + Q;
end