function pos = updatePlotLab3(fig, tip, R, S, T, V, W, X, curTime, returnPacket)
%updatePlotLab3 
%Updates the plots of the arm and its variables

currentAngle = processStatus(returnPacket);

pos= calcJointPos(currentAngle);
xPos = pos(1,:);
yPos = pos(2,:);
zPos = pos(3,:);

jointAngle1 = cast(currentAngle(1), 'double');
jointAngle2 = cast(currentAngle(2), 'double');
jointAngle3 = cast(currentAngle(3), 'double');

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));
addpoints(R, curTime, jointAngle1);
addpoints(S, curTime, jointAngle2);
addpoints(T, curTime, jointAngle3);

addpoints(V, curTime, double(xPos(4)));
addpoints(X, curTime, double(yPos(4)));
addpoints(W, curTime, double(zPos(4)));

set(fig.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);

drawnow();
end

