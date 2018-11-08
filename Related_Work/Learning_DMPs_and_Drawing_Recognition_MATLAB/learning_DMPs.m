%%% Visualize the forcing function of DMPs

tau = 1;
alpha = 1;

%% Define the phase variable z as a function of the time t
z = @(t) exp(-tau*alpha*t);

t = 0:0.1:10;

T = length(t);

Z = zeros(T,1);

for i = 1:T
    Z(i) = z(t(i));
end

% plot(t,Z);

h = 1;

%% Define some basis functions
phi1 = @(z) exp(-0.5*(z-0)^2/h); 
phi2 = @(z) exp(-0.5*(z-0.2)^2/h);
phi3 = @(z) exp(-0.5*(z-0.4)^2/h);
phi4 = @(z) exp(-0.5*(z-0.6)^2/h);
phi5 = @(z) exp(-0.5*(z-0.8)^2/h);

PHI1 = zeros(T,1);
PHI2 = zeros(T,1);
PHI3 = zeros(T,1);
PHI4 = zeros(T,1);
PHI5 = zeros(T,1);

for i = 1:T
    PHI1(i) = phi1(Z(i));
    PHI2(i) = phi2(Z(i));
    PHI3(i) = phi3(Z(i));
    PHI4(i) = phi4(Z(i));
    PHI5(i) = phi5(Z(i));
end

% hold all;
% plot(t,PHI1);
% plot(t,PHI2);
% plot(t,PHI3);
% plot(t,PHI4);
% plot(t,PHI5);

%% Define PSIs

SUM_PHIs = sum([PHI1'; PHI2'; PHI3'; PHI4'; PHI5']);

PSI1 = (PHI1.*Z)./SUM_PHIs';
PSI2 = (PHI2.*Z)./SUM_PHIs';
PSI3 = (PHI3.*Z)./SUM_PHIs';
PSI4 = (PHI4.*Z)./SUM_PHIs';
PSI5 = (PHI5.*Z)./SUM_PHIs';

% hold all;
% plot(t,PSI1);
% plot(t,PSI2);
% plot(t,PSI3);
% plot(t,PSI4);
% plot(t,PSI5);


%% Define the forcing function

f = sum([1*PSI1'; 1*PSI2'; 1*PSI3'; 1*PSI4'; 1*PSI5']);

plot(t,f);

