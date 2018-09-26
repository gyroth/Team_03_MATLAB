function pos = updatePlotLab4(fig, tip, quiv, V, W, X, Y, Z, AA, BB, CC, DD, curTime,xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, tVel)
%updatePlotLab4
%Updates the plots of the arm and its variables, adjusted for forward
%differential kinematics, and updates the velocity vector

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));

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
set(quiv.handle, 'xdata', xPos(4), 'ydata', yPos(4), 'zdata', zPos(4), 'udata', tVel(1,1), 'vdata', tVel(2,1), 'wdata', tVel(3,1));
drawnow();
end