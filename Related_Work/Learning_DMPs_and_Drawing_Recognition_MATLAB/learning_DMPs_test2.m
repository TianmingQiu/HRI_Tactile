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

f = sum([PHI1'; PHI2'; PHI3'; PHI4'; PHI5'; PHI6']);
f = f./max(f);

hold all;
plot(t,PHI1);
plot(t,PHI2);
plot(t,PHI3);
plot(t,PHI4);
plot(t,PHI5);
plot(t,PHI6);
plot(t,f);