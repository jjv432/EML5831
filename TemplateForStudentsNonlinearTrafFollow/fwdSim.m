function robot = fwdSim(robot,dt)

    robot.X = robot.X + robot.vel*cos(robot.Phi)*dt;
    robot.Y = robot.Y + robot.vel*sin(robot.Phi)*dt;
    robot.Phi = robot.Phi + robot.angVel*dt;
