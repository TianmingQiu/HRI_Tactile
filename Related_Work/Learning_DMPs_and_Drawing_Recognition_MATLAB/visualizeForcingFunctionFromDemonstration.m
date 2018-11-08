%%% Visualize forcing functions of real demonstrations

recordTrajectory;

alpha = 1;
beta = 1;

% forcing function x (fx)
gx = x(end,1);
fx = ddx - alpha*beta*(repmat(gx,size(x,1),1)-x) + alpha*dx;

% forcing function y (fy)
gy = y(end,1);
fy = ddy - alpha*beta*(repmat(gy,size(y,1),1)-y) + alpha*dy;

figure(2);
plot(fx);

% figure(3);
% plot(fy);