function [omega, gamma, error] = my_controller(robot, des, old_error, dt, pos)
    pos
  kp = pos(1);
  kd = pos(2);
  % ki = gains(3);

  error = des.Y - (robot.Y + (robot.width*sin(robot.Phi)));
  %error_sum = error_sum + error * dt;
  
  gamma = kp*error + kd*(error-old_error)/dt; %+ ki*error_sum;

  % gamma = gamma + drift;
  
%gamma = atan2(sin(gamma), cos(gamma));

  if (gamma>=pi/3)
      gamma = pi/3;
  elseif (gamma<=-pi/3)
          gamma=-pi/3;
  end

 
  omega = (robot.Vel/robot.width) * tan(gamma);


end