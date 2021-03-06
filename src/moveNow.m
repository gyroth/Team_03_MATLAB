function [ Ftip ] = moveNow(startPos,endPos,pp,packet, quiv,p)
%moveNow Moves the robot from the starting position to the ending position
%using trajectory generation. All numerical entries are in ticks
    PID_SERV_ID = 37;
    traveltime = 6;
%ticks, ticks/s, ADC bits
    returnPacket = getStatus(pp, packet);
    
    %Joint angles(deg)
    currentAngle = processStatus(returnPacket);
    
    %Answers where is the tip right now
    pos= calcJointPos(currentAngle);
    
    %calculates the coefficients for the trajectory of the arm from
    %"HOME" to above object (same Z plane)
    joint1TrajCoef = quintTraj(0,traveltime,0,0,startPos(1),endPos(1),0,0);
    joint2TrajCoef = quintTraj(0,traveltime,0,0,startPos(2),endPos(2),0,0);
    joint3TrajCoef = quintTraj(0,traveltime,0,0,startPos(3),endPos(3),0,0);
    
    moveStart = clock;
    
    Ftip = 0;
    
while(~reachedSetpoint(pos(:,4),endPos)&&(abs(etime(clock,moveStart))<traveltime+1))
    %ticks, ticks/s, ADC bits
    returnPacket = getStatus(pp, packet);
    
    %Joint angles(deg)
    currentAngle = processStatus(returnPacket);
    %ADC bits
    currentTor = processStatusTor(returnPacket);
    %Joint Torque (1x3 Matrix)
    appTorque = appliedTorque(currentTor); %%what measurement is this
    %torque in
    
    %Force at Tip
    Ftip = statics3001(currentAngle', appTorque'); %%what measurement
    %is this force in
    
    %Answers where is the tip right now
    pos= calcJointPos(currentAngle);
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    xTip = xPos(4);
    yTip = yPos(4);
    zTip = zPos(4);
    
    %Updates the stickplot with the new position and force vector of the
    %robot
    set(p.handle,'xdata', xPos,'ydata',yPos,'zdata', zPos);
    set(quiv.handle,'xdata', xTip,'ydata',yTip,'zdata', zTip,'udata', Ftip(1,1), 'vdata', Ftip(2,1), 'wdata', Ftip(3,1));
    
    %joint 1 trajectory points
    J1 = quintPoint(etime(clock,moveStart), joint1TrajCoef(1,1), joint1TrajCoef(2,1), joint1TrajCoef(3,1), joint1TrajCoef(4,1), joint1TrajCoef(5,1), joint1TrajCoef(6,1));
    %joint 2 trajectory points
    J2 = quintPoint(etime(clock,moveStart), joint2TrajCoef(1,1), joint2TrajCoef(2,1), joint2TrajCoef(3,1), joint2TrajCoef(4,1), joint2TrajCoef(5,1), joint2TrajCoef(6,1));
    %joint 3 trajectory points
    J3 = quintPoint(etime(clock,moveStart), joint3TrajCoef(1,1), joint3TrajCoef(2,1), joint3TrajCoef(3,1), joint3TrajCoef(4,1), joint3TrajCoef(5,1), joint3TrajCoef(6,1));
    
    movehere = [J1,J2,J3];
   
    pidPacket(1:3) = movehere;
    
    
    pp.write(PID_SERV_ID, pidPacket);
end

end

