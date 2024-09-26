%% Hw 1
%Due 9/27/24
%Jack Vranicar
%jjv20@fsu.edu

%{
    Implement the PSO algorithm to minimize the following function
    f(x,y) = (x − 4)^2 − 7 ∗ cos(2pi(x − 4)) + y2 − 7cos(2piy)
    with x E [−7 7] and y E [−7 7]
%}

%https://www.mathworks.com/matlabcentral/answers/66763-generate-random-numbers-in-range-from-0-8-to-4

%r = a + (b-a).*rand(100,1); from source above
%%
clc
clearvars
format compact
close all

% Call the PSO function
global_cost_vars = my_PSO;

%%
%
% <include>PSOParams.m</include> 
%
% <include>my_PSO.m</include>
%
% <include>hwFn.m</include>
%