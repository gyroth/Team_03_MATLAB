%%
% RBE3001 - Final Project
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
    %ticks, ticks/s, force measure
    returnPacket = getStatus(pp,packet);
    
    %current joint angles (deg)
    currentAngle = processStatus(returnPacket);
    %current joint velocities (deg/s)
    currentVel = processStatusVel(returnPacket);
    %current joint torques (ADC bit)
    currentTor = processStatusTor(returnPacket);
    appTorque = appliedTorque(currentTor);
    
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
    
    
    
    Ftip = statics3001(currentAngle', appTorque');
    
    %Creates stickplot model in the x-z plane
    hold on
    P = createStickPlot(xPos,yPos,zPos);
    quiv.handle = quiver3(double(xPos(4)),double(yPos(4)),double(zPos(4)),Ftip(1), Ftip(2), Ftip(3), 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScaleFactor', 15000, 'Color', 'b');
    %A.handle = plot([xPos(1,4),xPos(1,4)],[zPos(1,4),zPos(1,4)]);
    hold off
    drawnow();
    
    %time
    start = clock;
    
    %initializes the sending packet to zeros
    pidPacket = zeros(1, 15, 'single');
    
    while(1)
            %%ticks, ticks/s, ADC bits
            returnPacket = getStatus(pp, packet);
            
            %Joint angles(deg)
            currentAngle = processStatus(returnPacket);
            
            %ADC bits
            currentTor = processStatusTor(returnPacket);
            %Joint Torque
            appTorque = appliedTorque(currentTor);
            %Force at Tip
            Ftip = statics3001(currentAngle', appTorque');
            
            pos= calcJointPos(currentAngle);

            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
            
            xPos = pos(1,:);
            yPos = pos(2,:);
            zPos = pos(3,:);
            
            set(quiv.handle, 'xdata', xPos(4), 'ydata', yPos(4), 'zdata', zPos(4), 'udata', Ftip(1), 'vdata', Ftip(2), 'wdata', Ftip(3));
            set(P.handle,'xdata', xPos, 'ydata', yPos, 'zdata', zPos);
       
            drawnow();
        
            pause(.001);
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()