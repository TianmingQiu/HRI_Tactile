%%% visualize a PD controller

% initial position
s0 = 0;

% initial velocity
v0 = 0;

% desired position
sd = 100;

% desired velocity
vd = 0;

% parameters of the PD controller
alpha = 10;
beta = 3;

st = s0;
vt = v0;

t = 0:0.1:100;

T = length(t);

S = zeros(T,1);
A = zeros(T,1);

% simulation
for i = 1:T
   
   S(i) = st; 
    
   a = alpha*(sd-st) + beta*(vd-vt);
   
   A(i) = a;
    
   %new position
   st = st + vt*0.1 + a*0.1^2/2;
   
   %new velocity
   vt = vt + a*0.1;
    
    
end

plot(t,A);