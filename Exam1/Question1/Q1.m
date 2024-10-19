%clearvars -except Dynamic StaticA StaticB StaticC
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
title("Static A")

figure()
hold on
plot(StaticB.Time, StaticB.Acceleration.X)
plot(StaticB.Time, StaticB.Acceleration.Y)
plot(StaticB.Time, StaticB.Acceleration.Z)
legend("X", "Y", "Z")
title("Static B")

figure()
hold on
plot(StaticC.Time, StaticC.Acceleration.X)
plot(StaticC.Time, StaticC.Acceleration.Y)
plot(StaticC.Time, StaticC.Acceleration.Z)
legend("X", "Y", "Z")
title("Static C")

% Body Rates
figure()
hold on
plot(StaticA.Time, StaticA.Orientation.X)
plot(StaticA.Time, StaticA.Orientation.Y)
plot(StaticA.Time, StaticA.Orientation.Z)
legend("X", "Y", "Z")
title("Static A")

figure()
hold on
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.X)
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.Y)
plot(StaticB.Time(1:(end-1)), StaticB.Orientation.Z)
legend("X", "Y", "Z")
title("Static B")

figure()
hold on
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.X)
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.Y)
plot(StaticC.Time(1:(end-2)), StaticC.Orientation.Z)
legend("X", "Y", "Z")
title("Static C")

%% 1 B

