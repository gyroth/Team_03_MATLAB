function pos = updatePlotLab3(fig, tip, returnPacket)
%updatePlotLab3 
%Updates the plots of the arm and its variables

currentAngle = processStatus(returnPacket);

pos= calcJointPos(currentAngle);
xPos = pos(1,:);
yPos = pos(2,:);
zPos = pos(3,:);

addpoints(tip, double(xPos(4)), double(yPos(4)), double(zPos(4)));

set(fig.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);

drawnow();
end

