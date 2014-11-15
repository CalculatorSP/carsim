close all
clear all
clc
format shortg

%% Constants

rho = .00236; %density of air [slugs/ft^3]
W = 600; %weight of vehicle with driver [lbs]
g = 32.2; %ft/s^2
m = W/g; %slug
Cd = .5; %coefficient of drag [unitless]
tireDiameter = 10/12; %tire diameter [ft]
tireRadius = tireDiameter/2; %tire radius [ft]
A = 13.8; %frontal area [ft^2]

l = 71.26/12;  %wheelbase ft
WdistF = .4;
l1 = (1-WdistF)*l; %length front to cg   ft
l2 = l-l1;     %length cg to rear    ft
hCG = 13/12;   %height CG ft
uFt = 1.2;

first = 2.750;  %first gear ratio
second = 2.000; %second gear ratio
third = 1.667;  %third gear ratio
fourth = 1.444; %fourth gear ratio
fifth = 1.304;  %fifth gear ratio
sixth = 1.208;  %sixth gear ratio

gears = [first second third fourth fifth sixth];

primarydrive = 2.111;
finaldrive = 1.6; %final drive ratio

%% Import data

selectedRPM = [1 2000 8000 10250 14000];
selectedTorque = [11 20 40 42.6 33];
interpSteps = 1:14000;
linearInterp = interp1(selectedRPM,selectedTorque,interpSteps,'Linear');
cubicInterp = interp1(selectedRPM,selectedTorque,interpSteps,'PCHIP');
splineInterp = interp1(selectedRPM,selectedTorque,interpSteps,'Spline');

torqueCurve = cubicInterp;

%% Calculations

n = 8000;
time = 1:n;
wheelSpeed = zeros(n,1);
RPM = zeros(n,1);
torque = zeros(n,1);
Ft = zeros(n,1);
Fd = zeros(n,1);
a = zeros(n,1);
vehicleVelocity = zeros(n,1);
distance = zeros(n,1);
chosenGear = ones(n,1);
jWheelSpeed = 0;
timePerShift = 50;
Power = zeros(n,1);
engageClutchTime = 500;
shiftTime = 500;
shiftTimer = 0;
Nr = zeros(n,1);

Nr(1) = (1-WdistF)*W;
distance(1) = 0;
vehicleVelocity(1) = 0;
wheelSpeed(1) = vehicleVelocity(1)/tireRadius;
chosenGear(1) = 1;
RPM(1) = length(torqueCurve);
torque(1) = torqueCurve(RPM(1));
Ft(1) = torque(1)*gears(chosenGear(1))*primarydrive*finaldrive/tireRadius;
Fd(1) = .5*rho*Cd*A*vehicleVelocity(1)^2;
Power(1) = RPM(1)*torque(1);
if Ft(1) > Nr(1)*uFt
    Ft(1) = Nr(1)*uFt;
end
a(1) = (Ft(1)-Fd(1))/m;

for i = 2:n
    vehicleVelocity(i) = a(i-1)*((time(i)-time(i-1))/1000)+vehicleVelocity(i-1);
    distance(i) = (vehicleVelocity(i)+vehicleVelocity(i-1))/2*((time(i)-time(i-1))/1000)+distance(i-1);
    wheelSpeed(i) = vehicleVelocity(i)/tireRadius;
    Fd(i) = .5*rho*Cd*A*vehicleVelocity(i)^2;
    % If we are shifting, keep a constant RPM and apply no torque
    if shiftTimer > 0
        shiftTimer = shiftTimer-1;
        chosenGear(i) = chosenGear(i-1);
        RPM(i) = RPM(i-1);
        torque(i) = 0;
        Ft(i) = 0;
    else
        Ft(i) = -inf;
        for gear = chosenGear(i-1):length(gears)
            jRPM = wheelSpeed(i)*gears(gear)*primarydrive*finaldrive*(60/(2*pi));
            if (jRPM > length(torqueCurve)) || (jRPM < 1)
                jTorque = 0;            
            else
                jTorque = torqueCurve(round(jRPM));
            end
            jFt = jTorque*gears(gear)*primarydrive*finaldrive/tireRadius;
            if jFt > Ft(i)
                Ft(i) = jFt;
                RPM(i) = jRPM;
                torque(i) = jTorque;
                chosenGear(i) = gear;
            end
        end
    end
    if chosenGear(i) > chosenGear(i-1)
        shiftTimer = shiftTime;
    end
    
    Nr(i) = ((W*l1)/l) + ((m*a(i-1)*hCG)/l);
    
    a(i) = (Ft(i)-Fd(i))/m;
end

cross = time(find(distance >= 246.06,1))