clc;
clear all
close all
format compact

%% 

Nstates = 1;
Ninputs = 0;
Nmeas = 1;

A = zeros(Nstates);
B = zeros(Nstates, Ninputs);
C = zeros(Nmeas, Nstates);

A = 1;
B = 0;
C = 1;
Q = 0.1; % process noise, how bad is my model?

sys.A = A;
sys.B = B;
sys.C = C;
sys.Q = Q;

% initial conditions (for the state)
xk  = 0; % guessing my position is zero, 'at the wall' for sonar example
Pk = 1e6 * eye(Nstates); % how good is that guess? ^ Covariance of the state

load("measurements.mat");
Uk = 0; % no controll inputs
Rk = .01; % sensor noise

% Run the kalman filter

for i = 1:length(measurements)

    % predict
    [xk, Pk] = predictKF(sys, xk, Uk, Pk);


    % meausrement

    yk = measurements(i); 
    [xk, Pk] = updateKF(sys, xk, Pk, yk, Rk);

    history_state(i,:) = xk;

end

figure();
plot(measurements, "g*");
hold on
plot(history_state, "-r");
