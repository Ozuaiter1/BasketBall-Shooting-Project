clc;
clear;
close all;
goalHeight = 10;%feet
robotHeight = 3;%feet
shotDistance = 24;%feet
ballSize = 29.5/12/pi;%feet
rimSize = 18/12; %feet
g = 32.17; %ft/s/s
angleShot = (45:5:65);%deg
weight = 1.4; %lbsf


syms initialVelocity real;

for i = 1:length(angleShot)
    eqn = tand(angleShot(i))*shotDistance - 0.5*g*(shotDistance/initialVelocity/cosd(angleShot(i)))^2 == goalHeight - robotHeight;
    slv = solve(eqn,initialVelocity);
    speed(i) = abs(double(slv(1)));
end
x = (0:0.1:shotDistance)';
y = zeros(shotDistance*10+1,length(angleShot));

for i = 1:length(angleShot)
    for j = 1:length(x)
        y(j,i) = tand(angleShot(i))*x(j) - 0.5*g*(x(j)/speed(i)/cosd(angleShot(i)))^2 + robotHeight;
        if(isnan(y(j,i)))
            y(j,i) = 0;
        end
    end
plot(x,y(:,i));
hold on
end
axis equal;


for i = 1:length(angleShot)
    offsetDistance = shotDistance + ballSize - rimSize;
    loweqn = tand(angleShot(i))*offsetDistance() - 0.5*g*(offsetDistance/initialVelocity/cosd(angleShot(i)))^2 == goalHeight - robotHeight;
    lowslv = solve(loweqn,initialVelocity);
    lowspeed(i) = abs(double(lowslv(1)));
    offsetDistance = shotDistance - ballSize + rimSize;
    higheqn = tand(angleShot(i))*offsetDistance - 0.5*g*(offsetDistance/initialVelocity/cosd(angleShot(i)))^2 == goalHeight - robotHeight;
    highslv = solve(higheqn,initialVelocity);
    highspeed(i) = abs(double(highslv(1)));
end
table = [lowspeed;speed;highspeed];
energy = 1.35582*weight/g*table(2,5)