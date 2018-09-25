function pos = updatePlotLab4Q7(fig, tip, quiv, V, W, X, Y, Z, AA, BB, curTime, xPos, yPos, zPos, xVelo, yVelo, zVelo, tVel, magnitudeV)
%updatePlotLab4Q7 
%Updates the plots of the arm and its variables

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));

addpoints(V, curTime, magnitudeV);

addpoints(X, curTime, double(yVelo));
addpoints(W, curTime, double(xVelo));
addpoints(Y, curTime, double(zVelo));

addpoints(AA, curTime, double(yPos(4)));
addpoints(Z, curTime, double(zPos(4)));
addpoints(BB, curTime, double(xPos(4)));


set(fig.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);
set(quiv.handle, 'xdata', xPos(4), 'ydata', yPos(4), 'zdata', zPos(4), 'udata', tVel(1,1), 'vdata', tVel(2,1), 'wdata', tVel(3,1));
drawnow();
end
