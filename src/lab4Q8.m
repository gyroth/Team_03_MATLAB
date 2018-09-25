function [ output_args ] = lab4Q8( xPos, zPos, desP, desV)
%lab4Q8 xPos is the joint positions in X
       %zPos is the joint positions in Z
       %desP is the desired point to move to
       %desV is the desired velocity at which to move to desP
curP = [180;0;-34];


P = createXZStickPlot(xPos,zPos);

inp = ginput(1);
desP(1) = inp(1);
desP(2) = 0;
desP(3) = inp(2);

b = 1;
dontExit = true;
while(b<=10)
    loopStartTime = clock;
while(dontExit)
    
    done = (curP >= (desP - [2;2;2]))
    if(done == [1;1;1])
        dontExit = false;
    end
    xPos
    zPos
    %updates the plot
    set(P.handle,'xdata', xPos, 'ydata', zPos);
    pause(.01);
    
    %gets the velocity vector between current position and desired position
    velVec = calcVelVec(curP,desP,desV)
    
    %Degrees
    curAngles = iKin(curP);
    
    %calculates the desired joint velocities
    jV = invVelKin(curAngles, velVec)
    
    loopEndTime = clock;
    curP
    %change in angle
    jAng = jV*abs(etime(loopEndTime,loopStartTime));
    
    loopStartTime = clock;
    
    %new angle
    nJA = jAng+curAngles;
    
    %change in joint space
    incrementalSP = calcJointPos(nJA)
    
    %new position in joint space    
    curP = incrementalSP(:,4)
    
    xPos = incrementalSP(1,:);
    zPos = incrementalSP(3,:);
end
b = b+1;
dontExit = true;
curP = desP;
done = [0;0;0];

inp = ginput(1);

disp(inp)

desP(1) = inp(1);
desP(2) = 0;
desP(3) = inp(2);
end

