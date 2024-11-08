function H = myhomography(px,py,qx,qy)
x = px;
y = py;

u = qx;
v = qy;


j = 1;
for i = 1:4
     
    A(j,:) = [-x(i) -y(i) -1 0 0 0 x(i)*u(i) y(i)*u(i) u(i)];
    A(j+1,:) =  [0 0 0 -x(i) -y(i) -1 x(i)*v(i) y(i)*v(i) v(i)];
    j = j+2;
end



[U,S,V] = svd(A);
h = V(:,end); % why is this the sln?
H = [h(1:3)'; h(4:6)'; h(7:9)'];
H = H./H(3,3);
