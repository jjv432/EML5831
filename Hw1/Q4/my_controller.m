function [omega, gamma, error] = my_controller(robot, des, old_error, dt, gains)

% K values
kp = gains(1);
kd = gains(2);
%ki = gains(3);

% Calculate error
error = des.Y - (robot.Y + (robot.width*sin(robot.Phi)));

% Don't allow Ki term to get too big
% if error_sum >= 100
%     error_sum = 0;
% else
%     error_sum = error_sum + error * dt;
% end

% Apply PID
gamma = kp*error + kd*(error-old_error)/dt; % + ki*error_sum;

% Introduce noise
% gamma = gamma + drift;

%gamma = atan2(sin(gamma), cos(gamma));

% Set bounds on steering angle
if (gamma>=pi/3)
    gamma = pi/3;
elseif (gamma<=-pi/3)
    gamma=-pi/3;
end

% Body angular rotation
omega = (robot.Vel/robot.width) * tan(gamma);


end