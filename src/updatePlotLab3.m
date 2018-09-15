function pos = updatePlotLab3(fig, tip, V, W, X, Y, Z, AA, BB, CC, DD, curTime,xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc)
%updatePlotLab3 
%Updates the plots of the arm and its variables

% jointAngle1 = cast(currentAngle(1), 'double');
% jointAngle2 = cast(currentAngle(2), 'double');
% jointAngle3 = cast(currentAngle(3), 'double');

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));
% addpoints(R, curTime, jointAngle1);
% addpoints(S, curTime, jointAngle2);
% addpoints(T, curTime, jointAngle3);

addpoints(V, curTime, double(xPos(4)));
addpoints(X, curTime, double(yPos(4)));
addpoints(W, curTime, double(zPos(4)));

addpoints(Y, curTime, double(xVel(4)));
addpoints(AA, curTime, double(yVel(4)));
addpoints(Z, curTime, double(zVel(4)));

addpoints(BB, curTime, double(xAcc(4)));
addpoints(DD, curTime, double(yAcc(4)));
addpoints(CC, curTime, double(zAcc(4)));

set(fig.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);

drawnow();
end

