%% Hw 1
%Due 9/27/24
%Jack Vranicar
%jjv20@fsu.edu

%{
    Implement the PSO algorithm to minimize the following function
    𝑓(𝑥, 𝑦) = (𝑥 − 4)2 − 7 ∗ 𝑐𝑜𝑠(2𝜋(𝑥 − 4)) + 𝑦2 − 7𝑐𝑜𝑠(2𝜋𝑦)
    with 𝑥 ∈ [−7 7] 𝑎𝑛𝑑 𝑦 ∈ [−7 7]
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