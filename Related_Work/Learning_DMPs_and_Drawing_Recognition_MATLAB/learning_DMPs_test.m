%%% Visualize the forcing function of DMPs

alpha_z = 1;

%% Define the phase variable z as a function of the time t
z = @(t) exp(-tau*alpha_z*t);

t = 0:0.1:10;

T = length(t);

Z = zeros(T,1);

for i = 1:T
    Z(i) = z(t(i));
end

%plot(t,Z);

h = 1;

%% Define some basis functions
phi1 = @(t) exp(-0.5*(t-0)^2/h); 
phi2 = @(t) exp(-0.5*(t-2)^2/h);
phi3 = @(t) exp(-0.5*(t-4)^2/h);
phi4 = @(t) exp(-0.5*(t-6)^2/h);
phi5 = @(t) exp(-0.5*(t-8)^2/h);
phi6 = @(t) exp(-0.5*(t-10)^2/h);

PHI1 = zeros(T,1);
PHI2 = zeros(T,1);
PHI3 = zeros(T,1);
PHI4 = zeros(T,1);
PHI5 = zeros(T,1);
PHI6 = zeros(T,1);


for i = 1:T
    PHI1(i) = phi1(t(i));
    PHI2(i) = phi2(t(i));
    PHI3(i) = phi3(t(i));
    PHI4(i) = phi4(t(i));
    PHI5(i) = phi5(t(i));
    PHI6(i) = phi6(t(i));
end

% hold all;
% plot(t,PHI1);
% plot(t,PHI2);
% plot(t,PHI3);
% plot(t,PHI4);
% plot(t,PHI5);
% plot(t,PHI6);

%% Define PSIs

PSI1 = (PHI1.*Z);
PSI2 = (PHI2.*Z);
PSI3 = (PHI3.*Z);
PSI4 = (PHI4.*Z);
PSI5 = (PHI5.*Z);
PSI6 = (PHI6.*Z);

% hold all;
% plot(t,PSI1);
% plot(t,PSI2);
% plot(t,PSI3);
% plot(t,PSI4);
% plot(t,PSI5);
% plot(t,PSI6);

%% Define our desired trajectory

% initial position
s0 = 0;

% initial velocity
v0 = 0;

% desired position
sd = 100;

% desired velocity
vd = 0;

% parameters of the PD controller
alpha = 1;
beta = 10;

st = s0;
vt = v0;

S = zeros(T,1); % positions
V = zeros(T,1); % velocities
A = zeros(T,1); % accelerations

% simulation
for i = 1:T
   
   S(i) = st;
   V(i) = vt;
    
   a = alpha*beta*(sd-st) + alpha*(vd-vt) + 100*rand*sin(pi*t(i));
   
   A(i) = a;
    
   %new position
   st = st + vt*0.1 + a*0.1^2/2;
   
   %new velocity
   vt = vt + a*0.1;
    
end

hold all;
plot(t,S);
%plot(t,V);
%plot(t,A);
%legend('positions', 'velocities', 'accelerations');

%% Compute the forcing function given the desired trajectory
f = A - alpha*beta*(ones(length(S),1)*sd - S) + alpha*V;

%% Find the right weights to define the DMP
w = ([PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6']*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6']')^-1*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6']*f;

%% Define the forcing function using the learned weights

f = [PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6']'*w;

%% Generate the trajectory using now the DMP

% initial position
s0 = 0;

% initial velocity
v0 = 0;

% desired position
sd = 100;

% desired velocity
vd = 0;

% parameters of the PD controller
alpha = 1;
beta = 10;

st = s0;
vt = v0;

S = zeros(T,1); % positions
V = zeros(T,1); % velocities
A = zeros(T,1); % accelerations

% simulation
for i = 1:T
   
   S(i) = st;
   V(i) = vt;
    
   a = alpha*beta*(sd-st) + alpha*(vd-vt) + f(i);
   
   A(i) = a;
    
   %new position
   st = st + vt*0.1 + a*0.1^2/2;
   
   %new velocity
   vt = vt + a*0.1;
    
end

plot(t,S);
legend('original', 'reproduction');
