%%
% RBE3001 - Laboratory 4
%
%
% ------------
% This MATLAB script creates a live stick model plot of the robotic arm and keeps
% track of the position, velocity, and acceleration of the tip. The code
% below attempts to move the robot to the point where the user had clicked
% using ginput. Despite our best efforts, our logic seemed sound, but the
% robot did not move.
%
%
clear
clc
clear java %#ok<CLJAVA>
%clear import;
clear classes; %#ok<CLCLS>
vid = hex2dec('3742');
pid = hex2dec('0007');
%disp (vid );
%disp (pid);
javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

kP1 = .0001;
kI1 = .0001;
kD1 = .002;

kP2 = .004;
kI2 = 0;
kD2 = .009;

kP3 = .004;
kI3 = 0;
kD3 = .009;

try
    packet = zeros(15, 1, 'single');
    PID_SERV_ID = 37;
    
    pidVal = [kP1, kI1, kD1; ...
        kP2, kI2, kD2; ...
        kP3, kI3, kD3];
    
    setPIDConstants(pp, pidVal);
    
    
    returnPacket = getStatus(pp, packet);
    calibrate(pp,packet);
    % Sets the received packet into a 1x3 matrix of joint angles
    while(1)
    %ticks, ticks/s, force measure
    returnPacket = getStatus(pp,packet);
    
    %current joint angles (deg)
    currentAngle = processStatus(returnPacket);
    %current joint velocities (deg/s)
    currentVel = processStatusVel(returnPacket);
    %current joint torques (ADC bit)
    currentTor = processStatusTor(returnPacket);
    appliedTorque(currentTor)
    end
    
    runstart = clock;
    %changes degrees to angles
    
    %outputs points in task space for each coord frame
    pos= calcJointPos(currentAngle);
    
    %initializes variables to 0
    xPos = zeros(3,1,'single');
    yPos = zeros(3,1,'single');
    zPos = zeros(3,1,'single');
    
    xVel = zeros(3,1,'single');
    yVel = zeros(3,1,'single');
    zVel = zeros(3,1,'single');
    
    xAcc = zeros(3,1,'single');
    yAcc = zeros(3,1,'single');
    zAcc = zeros(3,1,'single');
    curTime = 0;
    
    desP = zeros(3,1);
    
    %initializes tracker for number of ginputs
    b = 1;
    
    % Sets the desired travel velocity in mm/s
    desVel = 105;
    
    %Current tip velocities in mm/s
    xVelo = currentVel(1);
    yVelo = currentVel(2);
    zVelo = currentVel(3);
    
    %Current tip position in Cartesian space
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    %Creates stickplot model in the x-z plane
    hold on
    P = createXZStickPlot(xPos,zPos);
    
    A.handle = plot([xPos(1,4),xPos(1,4)],[zPos(1,4),zPos(1,4)]);
    hold off
    
    %time
    start = clock;
    
    %initializes the sending packet to zeros
    pidPacket = zeros(1, 15, 'single');
    
    %gets a user defined point from the graph in Cartesian space
    input = ginput(1);
    
    %puts the user defined input into a 3d space vector instead of 2d
    desP(1) = input(1);
    desP(2) = 0;
    desP(3) = input(2);
    
    while(b<5)
        loopStartTime = clock;
        %checks to see if the arm is at the desired position in the
        %Cartesian space
        while(~reachedSetpoint(pos(:,4),desP))
            
            %%ticks, ticks/s, force measure
            returnPacket = getStatus(pp, packet);
            
            %Joint angles(deg)
            currentAngle = processStatus(returnPacket)
            
            pos= calcJointPos(currentAngle);
            pos(2,4) = -pos(2,4);
            
            %pos is the current tip position; kXYZ is the desired tip
            %position; desVel is the desired velocity
            velVec = calcVelVec(pos(:,4),desP,desVel)
            
            posVec = desP
            
            set(A.handle,'xdata',[pos(1,4),posVec(1)], 'ydata', [pos(3,4),posVec(3)]);
            drawnow()
            
            %Inverse Velocity Kinematics: returns joint velocities in
            %degrees
            jVel = invVelKin(currentAngle',velVec);
            
            loopEndTime = clock;
            
            %change in angle degrees
            jAng = jVel*abs(etime(loopEndTime,loopStartTime))
            
            loopStartTime = clock;
            
            %new angle in degrees
            incrementalSP = jAng' + currentAngle
            
            %new angle in ticks
            pidPacket(1:3) = incrementalSP*1024/90;
            
            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
            
            xPos = pos(1,:);
            zPos = pos(3,:);
            
            set(P.handle,'xdata', xPos, 'ydata', zPos);
            pause(.01);
            drawnow();
        end
        b = b+1;
        
        input = ginput(1);
        
        disp(input)
        
        desP(1) = input(1);
        desP(2) = 0;
        desP(3) = input(2);
        pause(.01);
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()