% recognize what the drawing is

recordTrajectory;

alpha = 10;
beta = 1;

% forcing function x (fx)
gx = x(end,1);
fx = ddx - alpha*beta*(repmat(gx,size(x,1),1)-x) + alpha*dx;

% forcing function y (fy)
gy = y(end,1);
fy = ddy - alpha*beta*(repmat(gy,size(y,1),1)-y) + alpha*dy;

% load mean values and variances correspondent to forcing functions fx from
% different models
heart_m1 = load('./Models/Heart/heart_m1.mat');
heart_D1 = load('./Models/Heart/heart_D1.mat');

rectangle_m1 = load('./Models/Rectangle/rectangle_m1.mat');
rectangle_D1 = load('./Models/Rectangle/rectangle_D1.mat');

a_m1 = load('./Models/A/a_m1.mat');
a_D1 = load('./Models/A/a_D1.mat');

b_m1 = load('./Models/B/b_m1.mat');
b_D1 = load('./Models/B/b_D1.mat');

c_m1 = load('./Models/C/c_m1.mat');
c_D1 = load('./Models/C/c_D1.mat');

stickman_m1 = load('./Models/Stickman/stickman_m1.mat');
stickman_D1 = load('./Models/Stickman/stickman_D1.mat');

circle_m1 = load('./Models/Circle/circle_m1.mat');
circle_D1 = load('./Models/Circle/circle_D1.mat');

% load mean values and variances correspondent to forcing functions fy from
% different models
heart_m2 = load('./Models/Heart/heart_m2.mat');
heart_D2 = load('./Models/Heart/heart_D2.mat');

rectangle_m2 = load('./Models/Rectangle/rectangle_m2.mat');
rectangle_D2 = load('./Models/Rectangle/rectangle_D2.mat');

a_m2 = load('./Models/A/a_m2.mat');
a_D2 = load('./Models/A/a_D2.mat');

b_m2 = load('./Models/B/b_m2.mat');
b_D2 = load('./Models/B/b_D2.mat');

c_m2 = load('./Models/C/c_m2.mat');
c_D2 = load('./Models/C/c_D2.mat');

stickman_m2 = load('./Models/Stickman/stickman_m2.mat');
stickman_D2 = load('./Models/Stickman/stickman_D2.mat');

circle_m2 = load('./Models/Circle/circle_m2.mat');
circle_D2 = load('./Models/Circle/circle_D2.mat');


% Use linear interpolation to make the drawings look like they took the
% same time, in order to compare the drawings later.
% Obs.: We assume here that the most important thing when comparing
% drawings is the shape of the forcing function of each drawing, not the length of time of each
% drawing.
timeLength = min([length(heart_m1.m1), length(rectangle_m1.m1), length(a_m1.m1), length(b_m1.m1), length(c_m1.m1), length(stickman_m1.m1), length(circle_m1.m1), length(fx)]); % in the following, it's as if all drawings had
% the length of time of the drawing that took less time
tLengthHeart = length(heart_m1.m1);
tLengthRectangle = length(rectangle_m1.m1);
tLengthA = length(a_m1.m1);
tLengthB = length(b_m1.m1);
tLengthC = length(c_m1.m1);
tLengthStickman = length(stickman_m1.m1);
tLengthCircle = length(circle_m1.m1);
tLengthF = length(fx);

timeStampsHeart = linspace(0,1,tLengthHeart);
timeStampsRectangle = linspace(0,1,tLengthRectangle);
timeStampsA = linspace(0,1,tLengthA);
timeStampsB = linspace(0,1,tLengthB);
timeStampsC = linspace(0,1,tLengthC);
timeStampsStickman = linspace(0,1,tLengthStickman);
timeStampsCircle = linspace(0,1,tLengthCircle);
timeStampsF = linspace(0,1,tLengthF);

timeStampsReference = linspace(0,1,timeLength);

newHeartFx = interp1(timeStampsHeart, heart_m1.m1, timeStampsReference);
newRectangleFx = interp1(timeStampsRectangle, rectangle_m1.m1, timeStampsReference);
newAFx = interp1(timeStampsA, a_m1.m1, timeStampsReference);
newBFx = interp1(timeStampsB, b_m1.m1, timeStampsReference);
newCFx = interp1(timeStampsC, c_m1.m1, timeStampsReference);
newStickmanFx = interp1(timeStampsStickman, stickman_m1.m1, timeStampsReference);
newCircleFx = interp1(timeStampsCircle, circle_m1.m1, timeStampsReference);

newHeartFy = interp1(timeStampsHeart, heart_m2.m2, timeStampsReference);
newRectangleFy = interp1(timeStampsRectangle, rectangle_m2.m2, timeStampsReference);
newAFy = interp1(timeStampsA, a_m2.m2, timeStampsReference);
newBFy = interp1(timeStampsB, b_m2.m2, timeStampsReference);
newCFy = interp1(timeStampsC, c_m2.m2, timeStampsReference);
newStickmanFy = interp1(timeStampsStickman, stickman_m2.m2, timeStampsReference);
newCircleFy = interp1(timeStampsCircle, circle_m2.m2, timeStampsReference);

newFx = interp1(timeStampsF, fx, timeStampsReference);
newFy = interp1(timeStampsF, fy, timeStampsReference);
newX = interp1(timeStampsF, x, timeStampsReference);
newY = interp1(timeStampsF, y, timeStampsReference);

% plot forcing function fx and means from the models
figure(2);
hold all;
plot(newFx);
plot(newHeartFx);
plot(newRectangleFx);
plot(newAFx);
plot(newBFx);
plot(newCFx);
plot(newStickmanFx);
plot(newCircleFx);
legend('new fx', 'heart', 'rectangle', 'a', 'b', 'c', 'stickman', 'circle');

% plot forcing function fy and means from the models
figure(3);
hold all;
plot(newFy);
plot(newHeartFy);
plot(newRectangleFy);
plot(newAFy);
plot(newBFy);
plot(newCFy);
plot(newStickmanFy);
plot(newCircleFy);
legend('new fy', 'heart', 'rectangle', 'a', 'b', 'c', 'stickman', 'circle');

% for each time step, compute the euclidian distance between the new
% drawing and the mean values of the models and compute the cumulative
% difference in relation to each model
cumDifferenceToHeart = zeros(1, timeLength + 1);
cumDifferenceToRectangle = zeros(1, timeLength + 1);
cumDifferenceToA = zeros(1, timeLength + 1);
cumDifferenceToB = zeros(1, timeLength + 1);
cumDifferenceToC = zeros(1, timeLength + 1);
cumDifferenceToStickman = zeros(1, timeLength + 1);
cumDifferenceToCircle = zeros(1, timeLength + 1);

differenceToHeart = 0;
differenceToRectangle = 0;
differenceToA = 0;
differenceToB = 0;
differenceToC = 0;
differenceToStickman = 0;
differenceToCircle = 0;

for i = 1:timeLength
    differenceToHeart = norm(newFx(i) - newHeartFx(i)) + norm(newFy(i) - newHeartFy(i));
    differenceToRectangle = norm(newFx(i) - newRectangleFx(i)) + norm(newFy(i) - newRectangleFy(i));
    differenceToA = norm(newFx(i) - newAFx(i)) + norm(newFy(i) - newAFy(i));
    differenceToB = norm(newFx(i) - newBFx(i)) + norm(newFy(i) - newBFy(i));
    differenceToC = norm(newFx(i) - newCFx(i)) + norm(newFy(i) - newCFy(i));
    differenceToStickman = norm(newFx(i) - newStickmanFx(i)) + norm(newFy(i) - newStickmanFy(i));
    differenceToCircle = norm(newFx(i) - newCircleFx(i)) + norm(newFy(i) - newCircleFy(i));
    
    cumDifferenceToHeart(i+1) = cumDifferenceToHeart(i) + differenceToHeart;
    cumDifferenceToRectangle(i+1) = cumDifferenceToRectangle(i) + differenceToRectangle;
    cumDifferenceToA(i+1) = cumDifferenceToA(i) + differenceToA;
    cumDifferenceToB(i+1) = cumDifferenceToB(i) + differenceToB;
    cumDifferenceToC(i+1) = cumDifferenceToC(i) + differenceToC;
    cumDifferenceToStickman(i+1) = cumDifferenceToStickman(i) + differenceToStickman;
    cumDifferenceToCircle(i+1) = cumDifferenceToCircle(i) + differenceToCircle;
end

figure(4);
axis([-1 1 -1 1]);
hold on;

plot0 = plot(NaN, '*-');

for timeStep = 1:timeLength
        
    set( plot0, 'XData', newX(1:timeStep) );
    set( plot0, 'YData', newY(1:timeStep) );
    
    drawnow;
    pause(0.005);
    
end

[minCumDifference, index] = min([cumDifferenceToHeart(end), cumDifferenceToRectangle(end), cumDifferenceToA(end), cumDifferenceToB(end), cumDifferenceToC(end), cumDifferenceToStickman(end), cumDifferenceToCircle(end)]);

switch index
    case 1
        title('HEART');
    case 2
        title('RECTANGLE');
    case 3
        title('A');
    case 4
        title('B');
    case 5
        title('C');
    case 6
        title('STICKMAN');
    case 7
        title('CIRCLE');
end

figure(5);
hold all;
plot(cumDifferenceToHeart);
plot(cumDifferenceToRectangle);
plot(cumDifferenceToA);
plot(cumDifferenceToB);
plot(cumDifferenceToC);
plot(cumDifferenceToStickman);
plot(cumDifferenceToCircle);
legend('cumulative difference to heart', 'cumulative difference to rectangle', 'cumulative difference to a', 'cumulative difference to b', 'cumulative difference to c', 'cumulative difference to stickman', 'cumulative difference to circle');
