%%
% RBE3001 - Laboratory 4
%
%
% ------------
% This MATLAB script creates a live stick model plot of the robotic arm and keeps
% track of the position, velocity, and acceleration of the tip. The code
% below has the robot moving its tip between given points using trajectory
% generation.
%
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
    
    returnPacket = getStatus(pp,packet);
    currentAngle = processStatus(returnPacket);
    
    runstart = clock;
    %changes degrees to angles
    
    pos= calcJointPos(currentAngle);
    
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
    
    %% Sets the position of the arm in task space
    
    % Waypoints in task space to create a triangle with the tip in millimeters
    viaPos = [[180;0;25],[170;-55;100], [170;55;100],[180;0;25]];
    
    % Waypoints to create a "3" with the tip in millimeters
    
    % Sets the desired travel velocities in task space for a downward
    % motion
    desVel = 45;
    
    viaJtsAngles = zeros(size(viaPos));
    
    numPoints = size(viaJtsAngles);
    
    for a = 1:numPoints(2)
        viaJtsAngles(:,a) = iKin(viaPos(:,a));
    end
    
    % Joint Angles in Ticks at each Setpoint
    viaJts = viaJtsAngles * 1024 / 90;
    
    previous = viaJts(:,1);
    
    returnPacket = getStatus(pp, packet);
    
    % Sets the received packet into a 3x3 matrix
    currentAngle = processStatus(returnPacket);
    currentVel = processStatusVel(returnPacket);
    
    pos= calcJointPos(currentAngle);
    
    xVelo = currentVel(1);
    yVelo = currentVel(2);
    zVelo = currentVel(3);
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    %Creates the stickplot model of the arm
    subplot(3,2,[1,3]);
    fig = createStickPlot(xPos, yPos, zPos);
    hold on
    quiv.handle = quiver3(double(xPos(4)),double(yPos(4)),double(zPos(4)),xVelo, yVelo, zVelo, 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScaleFactor', 0.04);
    tip = animatedline(double(xPos(4)),double(yPos(4)),double(zPos(4)), 'Color', 'g','LineWidth',1.5);
    hold off
    
    returnPacket = getStatus(pp, packet);
    
    magnitudeV = sqrt(power(double(xPos(4)),2)+power(double(yPos(4)),2)+power(double(zPos(4)),2));
    %Creates a plot of the magnitude of the tip velocity
    subplot(3,2,4); pause(.1);
    V = animatedline(etime(clock,runstart), magnitudeV, 'Color', [.196,.784,.235], 'LineWidth', 3);
    %Create title
    title({'Tip Velocity Magnitude'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    
    %Limit X
    xlim([0,25]);
    
    %Create ylabel
    ylabel({'Velocity(mm/s)'}, 'Color', [.196,.784,.235]);
    ylim([-400,400]);
    
    legend('Tip Magnitude', 'Location', 'northeast');
    
    %Creates a plot of the joint velocities of the arm
    subplot(3,2,6)
    W = animatedline(etime(clock,runstart), double(xVelo), 'Color', [.804,.216,.765], 'LineWidth',3);
    X =animatedline(etime(clock,runstart), double(yVelo), 'Color', [.196,.784,.235], 'LineWidth', 3);
    Y = animatedline(etime(clock,runstart), double(zVelo), 'Color', [.962,.145,.342], 'LineWidth', 3);
    %Create title
    title({'Joint Velocities'});
    %Create xlabel
    xlabel({'Time(s)'});
    
    %Limit X
    xlim([0,25]);
    
    %Create ylabel
    ylabel({'Velocities(deg/s)'}, 'Color', [.196,.784,.235]);
    ylim([-400,400]);
    
    legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'northeast');
    
    %Creates a plot for the velocities of the tip (Cartesian Velocity)
    subplot(3,2,2); pause(.1);
    %X Velocity
    BB = animatedline(etime(clock,runstart), double(xPos(4)), 'Color', [.196,.784,.235], 'LineWidth', 3);
    %Y Velocity
    Z = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    %Z Velocity
    AA = animatedline(etime(clock,runstart), double(yPos(4)), 'Color', [.962,.145,.342], 'LineWidth', 3);
    
    %Create title
    title({'Tip Velocity'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([0,25]);
    
    %Create ylabel
    ylabel({'Velocity(mm/sec)'}, 'Color', [.196,.784,.235]);
    ylim([-400,400]);
    
    legend('X Velocity', 'Y Velocity', 'Z Velocity', 'Location', 'northeast');
    
    
    %     %Create ylabel xVelo = currentVel(1);
    yVelo = currentVel(2);
    zVelo = currentVel(3);
    
    traveltime = 1.5;
    
    %Cycles through each point and moves the tip to the desired position
    %using trajectory generation. While moving, the program calls update
    %plot to make changes to the plots.
    for k = viaJts
        
        kAng = k * 90/1024;
        kXYZ = calcJointPos(kAng);
        kXYZ(2,4) = -kXYZ(2,4);
        
        last = previous;
        
        previous = k;
        
        start = clock;
        
        pidPacket = zeros(1, 15, 'single');
        
        loopStartTime = clock;
            while(~reachedSetpoint(pos(:,4),k))
                
                returnPacket = getStatus(pp, packet);
                
                prevTime = curTime;
                curTime = etime(clock, runstart);
                
                %Joint angles and joint velocities
                currentAngle = processStatus(returnPacket);
                currentVel  = processStatusVel(returnPacket);
                
                xVelo = currentVel(1);
                yVelo = currentVel(2);
                zVelo = currentVel(3);
                
                pos= calcJointPos(currentAngle);
                pos(2,4) = -pos(2,4);
                
                velVec = calcVelVec(pos(:,4),kXYZ(:,4),desVel);
                
                %Inverse Velocity Kinematics: returns joint velocities in
                %degrees
                jVel = invVelKin(currentAngle,velVec);
                
                loopEndTime = clock;
                %etime(loopEndTime,loopStartTime)
                jAng = jVel*abs(etime(loopEndTime,loopStartTime));
                loopStartTime = clock;
                
                incrementalSP = jAng' + currentAngle;
                
                pidPacket(1:3) = incrementalSP*1024/90;
                pp.write(PID_SERV_ID, pidPacket);
                pause(.004);
                pidReturnPacket = pp.read(PID_SERV_ID);
                
                % Calculates velocity and acceleration using changes in
                % position
                
                prevXPos = xPos;
                prevYPos = yPos;
                prevZPos = zPos;
                
                xPos = pos(1,:);
                yPos = pos(2,:);
                zPos = pos(3,:);
                
                magnitudeV = sqrt(power(double(xPos(4)),2)+power(double(yPos(4)),2)+power(double(zPos(4)),2));
                
                tVel = fwdVelKin(currentAngle,currentVel);
                
                prevXVel = xVel;
                prevYVel = yVel;
                prevZVel = zVel;
                
                xVel = (xPos-prevXPos)/(curTime - prevTime);
                yVel = (yPos-prevYPos)/(curTime - prevTime);
                zVel = (zPos-prevZPos)/(curTime - prevTime);
                
                xAcc = (xVel - prevXVel)/(curTime - prevTime);
                yAcc = (yVel - prevYVel)/(curTime - prevTime);
                zAcc = (zVel - prevZVel)/(curTime - prevTime);
                
                updatePlotLab4Q7(fig, tip, quiv, V, W, X, Y, Z, AA, BB, curTime, xPos, yPos, zPos, xVelo, yVelo, zVelo, tVel, magnitudeV);
                %updatePlotLab4(fig, tip, quiv, V, W, X, Y, Z, AA, BB, CC, DD, curTime, xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, tVel);
                
                %pause(.01);
                drawnow();
            end
    end
catch exception
    getReport(exception)
    %disp('Exited on error, clean shutdown');
end
pp.shutdown()