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

for i = 1:n
    if i <= engageClutchTime;
        RPM(i) = 2000;
        gear = 1;
        chosenGear(i) = gear;
        RPM(i) = min(RPM(i), length(torqueCurve));
        torque(i) = torqueCurve(round(RPM(i)));
        Ft(i) = torque(i)*gears(gear)*primarydrive*finaldrive/tireRadius;
        Fd(i) = .5*rho*Cd*A*vehicleVelocity(i)^2;
        Power(i) = RPM(i) * torque(i);
    else
        Ft(i) = -inf;
        for gear = chosenGear(i-1):length(gears)
            jWheelSpeed = vehicleVelocity(i)/tireRadius;
            jRPM = jWheelSpeed*gears(gear)*primarydrive*finaldrive*(60/(2*pi));
            jFd = .5*rho*Cd*A*vehicleVelocity(i)^2;
            if (jRPM > length(torqueCurve)) || (jRPM < 1) || (shiftTimer > 0)
                torque(i) = 0;
                Power(i) = 0;
                wheelSpeed(i) = jWheelSpeed;
                RPM(i) = jRPM;
                Ft(i) = 0;
                Fd(i) = jFd;
                chosenGear(i) = chosenGear(i-1);
            else
                jTorque = torqueCurve(round(jRPM));
                jFt = jTorque*gears(gear)*primarydrive*finaldrive/tireRadius;
                jPower = jRPM*jTorque;
                if jFt > Ft(i) && gear
                    Power(i) = jPower;
                    wheelSpeed(i) = jWheelSpeed;
                    RPM(i) = jRPM;
                    torque(i) = jTorque;
                    Ft(i) = jFt;
                    Fd(i) = jFd;
                    chosenGear(i) = gear;
                end
            end
        end
    end
    if (i >= 2) && (chosenGear(i) > chosenGear(i-1))
        shiftTimer = shiftTime;
    end
    if shiftTimer > 0
        shiftTimer = shiftTimer - 1;
    end
    
    if i >=2
        Nr(i) = ((W*l1)/l) + ((m*a(i-1)*hCG)/l);
    end
    if Ft(i) > uFt*Nr(i)
        Ft(i) = uFt*Nr(i);
    end
    
    a(i) = (Ft(i)-Fd(i))/m;
    
    if (i+1 <= n)
        vehicleVelocity(i+1) = a(i)*((time(i+1)-time(i))/1000)+vehicleVelocity(i);
        distance(i+1) = (vehicleVelocity(i)+vehicleVelocity(i+1))/2*((time(i+1)-time(i))/1000)+distance(i);
    end
    vehicleVelocity(n) = a(n)*((time(n)-time(n-1))/1000)+vehicleVelocity(n-1);
    distance(n) = vehicleVelocity(n)*((time(n)-time(n-1))/1000)+distance(n-1);
    %     if distance(i) >= 246.063
    %         break
    %     end
end

cross = time(find(distance >= 246.06,1))

%plot(time,distance,time(find(distance >= 246.06,1)),246,'r*');
%title('Distance vs. Time')
%xlabel('Time (s)')
%ylabel('Distance (ft)')