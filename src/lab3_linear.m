%%
% RBE3001 - Laboratory 3
%
%
% ------------
% This MATLAB script creates a live stick model plot of the robotic arm and keeps
% track of the position, velocity, and acceleration of the tip. The code
% below has the robot moving its tip between given points using linear
% interpolation.
%
%
clear
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
    % Sets the received packet into a 3x3 matrix
    currentAngle = processStatus(returnPacket);
    %     disp('return Pack')
    %     disp(returnPacket)
    runstart = clock;
     
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
    
    % Waypoints to create a triangle with the tip in millimeters
    viaPos = [[180;0;25],[45;-25;130], [50;25;120],[180;0;25]];
    
    %Waypoints to create a "3" with the tip in millimeters
    %viaPos = [[175;10;10],[175;-35;35],[175;-35;60],[175;0;80],[175;-35;105],[175;-35;130],[175;10;150]];
    
    %Linear Interpolation
    viaJtsAngles = zeros(3,12,3,'single');
    
    %Linear Interpolation
    for b = 1:3
        viaPnts(:,:,b) = interpolate(viaPos(:,b), viaPos(:,b+1));
    end
    % The joint angles to which to send the arm
    for c = 1:3
        for d = 1:12
            viaJtsAngles(:,d,c) = iKin(viaPnts(:,d,c));
        end
    end

viaJts = viaJtsAngles * 1024 / 90;

%Linear Interpolation
    previous = viaJts(:,1,1);
    
    returnPacket = getStatus(pp, packet);
    
    % Sets the received packet into a 3x3 matrix
    currentAngle = processStatus(returnPacket);
    %disp('return Pack')
    %disp(returnPacket)
    
    pos= calcJointPos(currentAngle);
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    subplot(3,2,[1,3]);
    fig = createStickPlot(xPos, yPos, zPos);
    tip = animatedline(double(xPos(4)),double(yPos(4)),double(zPos(4)), 'Color', 'g','LineWidth',1.5);
    
    returnPacket = getStatus(pp, packet);
    
    subplot(3,2,2); pause(.1);
    V = animatedline(etime(clock,runstart), double(xPos(4)), 'Color', [.196,.784,.235], 'LineWidth', 3);
    W = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    X = animatedline(etime(clock,runstart), double(yPos(4)), 'Color', [.962,.145,.342], 'LineWidth', 3);
    %Create title
    title({'Tip Positions'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([0,20]);
    
    %Create ylabel
    ylabel({'Position(mm)'}, 'Color', [.196,.784,.235]);
    ylim([-400,400]);
    
    legend('X Position', 'Y Position', 'Z Position', 'Location', 'northeast');
    
    subplot(3,2,4); pause(.1);
    Y = animatedline(etime(clock,runstart), double(xPos(4)), 'Color', [.196,.784,.235], 'LineWidth', 3);
    Z = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    AA = animatedline(etime(clock,runstart), double(yPos(4)), 'Color', [.962,.145,.342], 'LineWidth', 3);
    %Create title
    title({'Tip Velocity'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([0,20]);
    
    %Create ylabel
    ylabel({'Velocity(mm/sec)'}, 'Color', [.196,.784,.235]);
    ylim([-400,400]);
    
    legend('X Velocity', 'Y Velocity', 'Z Velocity', 'Location', 'northeast');
    
    subplot(3,2,6); pause(.1);
    BB = animatedline(etime(clock,runstart), double(xPos(4)), 'Color', [.196,.784,.235], 'LineWidth', 3);
    CC = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    DD = animatedline(etime(clock,runstart), double(yPos(4)), 'Color', [.962,.145,.342], 'LineWidth', 3);
    %Create title
    title({'Tip Acceleration'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([0,20]);
    
    %Create ylabel
    ylabel({'Acceleration(mm/sec^2)'}, 'Color', [.196,.784,.235]);
    ylim([-600,600]);
    
    legend('X Acceleration', 'Y Acceleration', 'Z Acceleration', 'Location', 'northeast');
    
    traveltime = .5;
    
    for k = viaJts
        
        last = previous;
        
        previous = k;
        
        start = clock;
        
        joint1TrajCoef = quintTraj(0,traveltime,0,0,last(1),k(1),0,0);
        joint2TrajCoef = quintTraj(0,traveltime,0,0,last(2),k(2),0,0);
        joint3TrajCoef = quintTraj(0,traveltime,0,0,last(3),k(3),0,0);  
        
        pidPacket = zeros(1, 15, 'single');
        
        
        %Cycles through each point and moves the tip to the desired position
        %using linear interpolation. While moving, the program calls update
        %plot to make changes to the plots.
        while(etime(clock,start)<traveltime)

            %joint 1 trajectory points
            J1 = quintPoint(etime(clock,start), joint1TrajCoef(1,1), joint1TrajCoef(2,1), joint1TrajCoef(3,1), joint1TrajCoef(4,1), joint1TrajCoef(5,1), joint1TrajCoef(6,1));
            %joint 2 trajectory points
            J2 = quintPoint(etime(clock,start), joint2TrajCoef(1,1), joint2TrajCoef(2,1), joint2TrajCoef(3,1), joint2TrajCoef(4,1), joint2TrajCoef(5,1), joint2TrajCoef(6,1));
            %joint 3 trajectory points
            J3 = quintPoint(etime(clock,start), joint3TrajCoef(1,1), joint3TrajCoef(2,1), joint3TrajCoef(3,1), joint3TrajCoef(4,1), joint3TrajCoef(5,1), joint3TrajCoef(6,1));          
            
            %I = [J1;J2;J3];
            
            %pidPacket(1:3) = [J1,J2,J3];
            
            pidPacket(1:3) = k;
            
            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
            
            returnPacket = getStatus(pp, packet);
            
            prevTime = curTime;
            curTime = etime(clock, runstart);
            
            currentAngle = processStatus(returnPacket);
            
            pos= calcJointPos(currentAngle);
            
            % Calculates velocity and acceleration using changes in
            % position
            
            prevXPos = xPos;
            prevYPos = yPos;
            prevZPos = zPos;
            
            xPos = pos(1,:);
            yPos = pos(2,:);
            zPos = pos(3,:);
            
            prevXVel = xVel;
            prevYVel = yVel;
            prevZVel = zVel;
            
            xVel = (xPos-prevXPos)/(curTime - prevTime);
            yVel = (yPos-prevYPos)/(curTime - prevTime);
            zVel = (zPos-prevZPos)/(curTime - prevTime);
            
            xAcc = (xVel - prevXVel)/(curTime - prevTime);
            yAcc = (yVel - prevYVel)/(curTime - prevTime);
            zAcc = (zVel - prevZVel)/(curTime - prevTime);
            
            updatePlotLab3(fig, tip, V, W, X, Y, Z, AA, BB, CC, DD, curTime, xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc);
            
            drawnow();
        end
        
    end
catch exception
    getReport(exception)
    %disp('Exited on error, clean shutdown');
end
pp.shutdown()
