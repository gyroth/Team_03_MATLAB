function pos = updatePlot(R, S, returnPacket)
%Updates the stickplot of the arm

currentAngle = processStatus(returnPacket);

pos= drawArm_function(currentAngle);
xPos = pos(1,:);
yPos = pos(2,:);
zPos = pos(3,:);

addpoints(S, double(xPos(4)), double(yPos(4)), double(zPos(4))); 
set(R.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);
drawnow();
end

