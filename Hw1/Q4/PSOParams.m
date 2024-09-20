function [Params] = PSOParams()

%Number of Particles
Params.N = 50; 

%Search Boundaries
Params.ya = 0;
Params.yb = 7;
Params.xa = 0;
Params.xb = 7;

%Initializing particl name matrix
Params.paricle_names = zeros(1, Params.N);

%Initializing global values
Params.G.best_cost = 10;
Params.G.best_pos = [0 0];

%Setting alpha values, and alpha1 max and min
Params.alpha_1_max = 10;
Params.alpha_1_min = 1;
Params.alpha_1_delta = 0.5; %Chnage in alpha 1 each try

Params.alpha_2 = 5;
Params.alpha_3 = 6.5;

%Setting time variables (seconds)
Params.dt = 0.01;
Params.max_time = .5;

%Setting maximum velocity
Params.max_vel = 10;

%Initializing global cost matrix
Params.global_cost(1) = Params.G.best_cost;

%Plotting 
Params.plotBool = 1; %1 for plots, 0 for no plots
Params.plotResolution = 1;%How many frames to advance from the previous one

end