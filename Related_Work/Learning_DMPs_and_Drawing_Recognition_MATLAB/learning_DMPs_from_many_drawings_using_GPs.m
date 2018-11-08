%%% Learning DMPs from many drawings using GPs % TODO

recordTrajectory;

T = length(x);

alpha = 10;
beta = 1;

% forcing function x (fx)
gx = x(end);
fx = ddx - alpha*beta*(repmat(gx,size(x,1),1)-x) + alpha*dx;

% forcing function y (fy)
gy = y(end,1);
fy = ddy - alpha*beta*(repmat(gy,size(y,1),1)-y) + alpha*dy;

figure(2);
plot(fx);

% figure(3);
% plot(fy);



Xa = 1:T; % times where I am trying to predict the forcing function
Xb = 1:T; % times where I know the forcing function

X = [Xa, Xb];

l = size(Xa,2); % number of validation samples = number of values for the forcing function I'm trying to predict
m = size(Xb,2); % number of training samples
n = l+m;

y1a = zeros(1,l); % forcing function values I'm trying to predict
y1b = fx'; % forcing function values I've observed already

y2a = zeros(1,l); % forcing function values I'm trying to predict
y2b = fy'; % forcing function values I've observed already

%%%% Gaussian Process Regression

sigma_f = 1;
sigma_w = 0.01;
lambda_1 = 100;
lambda_2 = 100;

k1 = @(x,y) sigma_f^2*exp((-1/2)*(x-y)'*lambda_1*(x-y));
k2 = @(x,y) sigma_f^2*exp((-1/2)*(x-y)'*lambda_2*(x-y));

% Compute covariance matrices K1, where K1_ij = k1(x_i, x_j)
%                             K2, where K2_ij = k2(x_i, x_j)
for i = 1:n
    for j = 1:n
       disp([i,j]);
       K1(i,j) = k1(X(:,i),X(:,j));
       K2(i,j) = k2(X(:,i),X(:,j));
    end
end

K1aa = K1(1:l, 1:l);
K1ab = K1(1:l, l+1:l+m);
K1ba = K1(l+1:l+m, 1:l);
K1bb = K1(l+1:l+m, l+1:l+m);

K2aa = K2(1:l, 1:l);
K2ab = K2(1:l, l+1:l+m);
K2ba = K2(l+1:l+m, 1:l);
K2bb = K2(l+1:l+m, l+1:l+m);

% p(y1a | y1b) ~ N(y1a | m1, D1)

mu_a = mean(Xa);
mu_b = mean(Xb);

m1 = mu_a' + K1ab*inv(K1bb + sigma_w^2*eye(m))*(y1b' - mu_b');
D1 = (K1aa + sigma_w^2*eye(l)) - K1ab*inv(K1bb + sigma_w^2*eye(m))*K1ba;

m2 = mu_a' + K2ab*inv(K2bb + sigma_w^2*eye(m))*(y2b' - mu_b');
D2 = (K2aa + sigma_w^2*eye(l)) - K2ab*inv(K2bb + sigma_w^2*eye(m))*K2ba;

figure(3);
xlabel('time');
ylabel('forcing function');
hold on;
h1 = shadedErrorBar(1:T, m1, 2*sqrt(diag(D1)), '-r');
h2 = shadedErrorBar(1:T, fx, zeros(size(fx,1),1), '-b');
legend([h2.mainLine, h1.mainLine, h1.patch], 'ground truth', 'GP prediction mean', '95% confidence interval');

figure(4);
xlabel('time');
ylabel('forcing function');
hold on;
h3 = shadedErrorBar(1:T, m2, 2*sqrt(diag(D2)), '-r');
h4 = shadedErrorBar(1:T, fy, zeros(size(fy,1),1), '-b');
legend([h4.mainLine, h3.mainLine, h3.patch], 'ground truth', 'GP prediction mean', '95% confidence interval');


%% Generate the trajectory using now the DMP

% initial position
sx0 = x(1);
sy0 = y(1);

% initial velocity
vx0 = dx(1);
vy0 = dy(1);

% desired position
sxd = x(end);
syd = y(end);

% desired velocity
vxd = dx(end);
vyd = dy(end);

% parameters of the PD controller
alpha = 10;
beta = 1;

sxt = sx0;
vxt = vx0;

syt = sy0;
vyt = vy0;

Sx = zeros(T,1); % positions
Vx = zeros(T,1); % velocities
Ax = zeros(T,1); % accelerations

Sy = zeros(T,1); % positions
Vy = zeros(T,1); % velocities
Ay = zeros(T,1); % accelerations

%m1 = [m1; zeros(size(m1,1),1)]; % just extending the vector with the approximated forcing functions to make sure that this force is vanishing

% simulation
for i = 1:T
   
   Sx(i) = sxt;
   Vx(i) = vxt;
   
   Sy(i) = syt;
   Vy(i) = vyt;
    
   ax = alpha*beta*(sxd-sxt) + alpha*(vxd-vxt) + m1(i); % m1 has the most probable value for the forcing function
                                                        % for each time step
   ay = alpha*beta*(syd-syt) + alpha*(vyd-vyt) + m2(i);
   
   Ax(i) = ax;
   Ay(i) = ay;
    
   %new position
   sxt = sxt + vxt*0.2 + ax*0.2^2/2;
   syt = syt + vyt*0.2 + ay*0.2^2/2;
   
   %new velocity
   vxt = vxt + ax*0.2;
   vyt = vyt + ay*0.2;
    
end

figure(5);
hold all;
plot(x);
plot(Sx);
legend('original', 'reproduction');

figure(6);
hold all;
plot(y);
plot(Sy);
legend('original', 'reproduction');

figure(7);
hold all;
plot(x, y);
plot(Sx, Sy);
legend('original', 'reproduction');