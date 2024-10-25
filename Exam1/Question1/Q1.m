clearvars -except Dynamic StaticA StaticB StaticC
close all
clc
%% Load the data
dataBool = 1;

if dataBool

Dynamic = load("DYNAMIC TAPED.mat"); % use the proper filename
dateTime = Dynamic.Acceleration.Timestamp; % Extract the timestamp, which is both date and time
Time = timeofday(dateTime); % From the date and time, extract just time of day
Time = seconds(Time); % Convert time of day from hours, minutes, seconds, to just seconds
Dynamic.Time = Time-Time(1); % Offset everything by the first time value so we start at 0;
clear dateTime Time

StaticA = load("STATIC A.mat"); % use the proper filename
dateTime = StaticA.Acceleration.Timestamp; % Extract the timestamp, which is both date and time
Time = timeofday(dateTime); % From the date and time, extract just time of day
Time = seconds(Time); % Convert time of day from hours, minutes, seconds, to just seconds
StaticA.Time = Time-Time(1); % Offset everything by the first time value so we start at 0;
clear dateTime Time

StaticB = load("STATIC B.mat"); % use the proper filename
dateTime = StaticB.Acceleration.Timestamp; % Extract the timestamp, which is both date and time
Time = timeofday(dateTime); % From the date and time, extract just time of day
Time = seconds(Time); % Convert time of day from hours, minutes, seconds, to just seconds
StaticB.Time = Time-Time(1); % Offset everything by the first time value so we start at 0;
clear dateTime Time

StaticC = load("STATIC C.mat"); % use the proper filename
dateTime = StaticC.Acceleration.Timestamp; % Extract the timestamp, which is both date and time
Time = timeofday(dateTime); % From the date and time, extract just time of day
Time = seconds(Time); % Convert time of day from hours, minutes, seconds, to just seconds
StaticC.Time = Time-Time(1); % Offset everything by the first time value so we start at 0;
clear dateTime Time


end

%% 1 A

figure()
hold on
plot(StaticA.Time, StaticA.Acceleration.X)
plot(StaticA.Time, StaticA.Acceleration.Y)
plot(StaticA.Time, StaticA.Acceleration.Z)
legend("X", "Y", "Z")
title("Static A Accelerometer")
xlabel("Time (s)")
ylabel("Acceleration (m/s/s)")

figure()
hold on
plot(StaticB.Time, StaticB.Acceleration.X)
plot(StaticB.Time, StaticB.Acceleration.Y)
plot(StaticB.Time, StaticB.Acceleration.Z)
legend("X", "Y", "Z")
title("Static B Accelerometer")
xlabel("Time (s)")
ylabel("Acceleration (m/s/s)")

figure()
hold on
plot(StaticC.Time, StaticC.Acceleration.X)
plot(StaticC.Time, StaticC.Acceleration.Y)
plot(StaticC.Time, StaticC.Acceleration.Z)
legend("X", "Y", "Z")
title("Static C Accelerometer")
xlabel("Time (s)")
ylabel("Acceleration (m/s/s)")

% Body Rates
figure()
hold on
plot(StaticA.Time, StaticA.Orientation.X)
plot(StaticA.Time, StaticA.Orientation.Y)
plot(StaticA.Time, StaticA.Orientation.Z)
legend("X", "Y", "Z")
title("Static A Gyro")
xlabel("Time (s)")
ylabel("Angle (deg)")

figure()
hold on
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.X)
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.Y)
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.Z)
legend("X", "Y", "Z")
title("Static B Gyro")
xlabel("Time (s)")
ylabel("Angle (deg)")

figure()
hold on
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.X)
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.Y)
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.Z)
legend("X", "Y", "Z")
title("Static C Gyro")
xlabel("Time (s)")
ylabel("Angle (deg)")

%% 1 B

StaticAAccelX = mean(StaticA.Acceleration.X);
StaticAAccelY = mean(StaticA.Acceleration.Y);
StaticAAccelZ = mean(StaticA.Acceleration.Z);

StaticBAccelX = mean(StaticB.Acceleration.X);
StaticBAccelY = mean(StaticB.Acceleration.Y);
StaticBAccelZ = mean(StaticB.Acceleration.Z);

StaticCAccelX = mean(StaticC.Acceleration.X);
StaticCAccelY = mean(StaticC.Acceleration.Y);
StaticCAccelZ = mean(StaticC.Acceleration.Z);

thetaA = (180/pi) * atan2(StaticAAccelY, StaticAAccelZ);
thetaB = (180/pi) * atan2(StaticBAccelY, StaticBAccelZ);
thetaC = (180/pi) * atan2(StaticCAccelY, StaticCAccelZ);

%% 1 C

% Angle[k] = [theta_dot_gyro * dt + angle_[k-1]]G + theta_accel *
% (1-G);
temp = 319;
DynamicOrientationX = Dynamic.Orientation.X(1:temp);
DynamicOrientationY = Dynamic.Orientation.Y(1:temp);
DynamicOrientationZ = Dynamic.Orientation.Z(1:temp);

figure()
hold on
plot(DynamicOrientationX)
plot(DynamicOrientationY)
plot(DynamicOrientationZ)
%legend("X", "Y", "Z")

%Angle(1) = thetaA;
dt = Dynamic.Time(2) - Dynamic.Time(1);

Gs = [.99 .93 .82 .75];

for t = 1:length(Gs)

    G = Gs(t);

for k = 1:length(DynamicOrientationY)

    Angle(k) = -(DynamicOrientationY(k)) * G + Dynamic.Acceleration.Z(k) * (1-G);

end

figure()
plot(Dynamic.Time(1:k), -Angle)
title(strcat("Angle v Time for K = ", num2str(G)))
xlabel("time (s)")
ylabel("\theta (deg)")

end