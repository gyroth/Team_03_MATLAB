function pos = updatePlot(fig, tip, O,P,Q,R,S,T,V, W, I, curTime, returnPacket)
%function pos = updatePlot(fig, tip, O,P,Q,R,S,T,V, W,setpoint, I, curTime, returnPacket)

%Updates the stickplot of the arm

currentAngle = processStatus(returnPacket);

pos= drawArm_function(currentAngle);
xPos = pos(1,:);
yPos = pos(2,:);
zPos = pos(3,:);

currentAngle = processStatus(returnPacket);
jointAngle1 = cast(currentAngle(1), 'double');
jointAngle2 = cast(currentAngle(2), 'double');
jointAngle3 = cast(currentAngle(3), 'double');

velocity1 = cast(returnPacket(2), 'double')*90/1024;
velocity2 = cast(returnPacket(5), 'double')*90/1024;
velocity3 = cast(returnPacket(8), 'double')*90/1024;

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));
%set(setpoint.handle, 'xdata', I(1), 'ydata', I(2), 'zdata', I(3));

addpoints(V, curTime, double(xPos(4)));
addpoints(W, curTime, double(zPos(4)));

addpoints(R, curTime, jointAngle1);
addpoints(O, curTime, velocity1);
addpoints(S, curTime, jointAngle2);
addpoints(P, curTime, velocity2);
addpoints(T, curTime, jointAngle3);
addpoints(Q, curTime, velocity3);

set(fig.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);

drawnow();
end

