%% robot animation function

function drawRobot(hgTX,robot)

%     leftCornerBodyX = -(robot.length/2);
%     leftCornerBodyY = -(robot.bodyWidth/2);
%     % Parent a rectangle to an hgtransform
%     g = hgtransform;
%     robotBody = rectangle('Parent',g,'Position',[leftCornerBodyX leftCornerBodyY robot.length robot.bodyWidth],'FaceColor',[1 0 0])
%     shaftLength = 3.5/100;
%     leftCornerRightWheelX = leftCornerBodyX - (3/100);
%     leftCornerRightWheelY = leftCornerBodyY - shaftLength - robot.wheelWidth;
% 
%     leftCornerLeftWheelX = leftCornerBodyX - (3/100);
%     leftCornerLeftWheelY = leftCornerBodyY + robot.bodyWidth + shaftLength;
% 
%     wheelRight = rectangle('Parent',g,'Position',[leftCornerRightWheelX leftCornerRightWheelY  2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0])
%     wheelLeft = rectangle('Parent',g,'Position',[leftCornerLeftWheelX leftCornerLeftWheelY 2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0])
% 
%     %sensorHead = rectangle('Parent',g,'Position',[robot.length - sensorHead.width 0 sensorHead.width sensorHead.width])
%     %rectangle('Position',[1 2 5 6])
%     %g.Matrix = makehgtform('zrotate',pi/2) % apply the transform to the children
% 
%     %axis('equal')
%     % figure()
%     axis([-5 5 -5 5])
%     axis('equal')
%     
%     %g.Matrix = makehgtform('zrotate',pi/2) % apply the transform to the children
%     X = 1;Y = 1; Phi = pi/4;
      hgTX.Matrix = makehgtform('translate',[robot.X robot.Y 0],'zrotate',robot.Phi); 
%     grid on
%     % handleBody = rectangle('position',[1 2 5 6])
%     % for k = 1:5,
%     %     set(handleBody,'position',[1+k 2+k 5 6])
%     % 
%     % end