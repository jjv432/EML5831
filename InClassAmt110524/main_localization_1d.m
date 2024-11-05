Nc = 5;
p = (1/Nc) * ones(1,Nc);

world = ['b', 'r', 'r', 'b', 'b'];
measurements = ['r', 'b']; % hardcoded measurements, not realistic

for i = 1:2
    z = measurements(i); % take a measurement
    [p, q] = sense(world, p, z);
    p = move(p, 1);
end

function q = move(p, U)

pUndershoot = 0.1;
pExact = 0.8;
pOvershoot = 0.1;

qu = pUndershoot*circshift(p,U-1,2); % shifts elements, allows it to wrap around
qe = pExact*circshift(p,U,2);
qo = pOvershoot*circshift(p,U+1,2);

q = qu + qe + qo;
end



function [p, q] = sense(world, p, z)
%{
    Inputes: p: prior belief, z: measurement
    Outputs: q: non-normalized posterior p: normalized belief
%}

pHit = 0.6; % probability of a hit (right color)
pMiss = 0.2; % probability of a  miss (wrong color)
q = zeros(1,length(p));

for i = 1:length(world)
    hit = (z == world(i)); % if color is correct, hit is true
    q(i) = p(i) * (hit*pHit + (1-hit)*pMiss); %multiply by appropriate probability depending on if a hit or miss
end

p = q/sum(q); % normalizing the matrix (posterior belief)
end