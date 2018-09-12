%%
% RBE3001 - Laboratory 2
%
% 
% ------------
% This MATLAB script creates a live stick model plot of the robotic arm for
% lab 2 of RBE3001. The code below also uses a cubic trajectory function to plot points between
% four assigned points oriented in the X,Z plane. It also creates plots of the joint positions and velocities while 
% tracing the tip of the robot on the stick plot.
%
% 
clear
clear java %#ok<CLJAVA>
%clear import;
clear classes; %#ok<CLCLS>
vid = hex2dec('3742');
pid = hex2dec('0007');
disp (vid );
disp (pid);
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

try
    packet = zeros(15, 1, 'single');
    PID_SERV_ID = 37;
    runstart = clock;
    
    %% Sets the Waypoints of the arm in angles
    viaPtsAngles = [[0; 0; 0], [0; 15; 45], [0; 45; -10] , [0;0;0]];
    viaPts = [[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0]];
    viaPts = viaPtsAngles * 1024 / 90;
    
    previous = [0;0;0];
    
    returnPacket = getStatus(pp, packet);
    
    % Sets the received packet into a 3x3 matrix
    currentAngle = processStatus(returnPacket);
    disp('return Pack')
    disp(returnPacket)
    
    pos= calcJointPos(currentAngle);
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    subplot(3,2,[1,3]);
    fig = createStickPlot(xPos, yPos, zPos);
    
    tip = animatedline(double(xPos(4)),double(yPos(4)),double(zPos(4)), 'Color', 'g','LineWidth',1.5);
        
    joint1 = cast(currentAngle(1),'double');
    joint2 = cast(currentAngle(2),'double');
    joint3 = cast(currentAngle(3),'double');
    
    velocity1 = cast(returnPacket(2), 'double')*90/1024;
    velocity2 = cast(returnPacket(5), 'double')*90/1024;
    velocity3 = cast(returnPacket(8), 'double')*90/1024;
    
    %Joint1 plot
    subplot(3,2,2);
    R = animatedline(etime(clock,runstart), joint1, 'Color', 'r','LineWidth',3);
    xlim([10,50]);
    ylim([-90, 90]);
    
    % Create title
    title({'Joint 1'});
    % Create xlabel
    xlabel({'Time(s)'});
    % Create ylabel
    yyaxis left
    ylabel({'Angle(degrees)'}, 'Color', 'r');
    
    yyaxis right
    O = animatedline(etime(clock,runstart), velocity1, 'Color', 'k','LineWidth',3);
    ylabel({'Velocity(degrees/sec)'}, 'Color', 'k');
    ylim([-150, 150]);
    
    %Joint2 plot
    subplot(3,2,4); pause(.004);
    S = animatedline(etime(clock,runstart), joint2, 'Color', 'g','LineWidth',3);
    xlim([10,50]);
    ylim([-90, 90]);
    
    % Create title
    title({'Joint 2'});
    % Create xlabel
    xlabel({'Time(s)'});
    % Create ylabel
    yyaxis left
    ylabel({'Angle(degrees)'}, 'Color', 'g');
    
    yyaxis right
    P = animatedline(etime(clock,runstart), velocity2, 'Color', 'm','LineWidth',3);
    ylabel({'Velocity(degrees/sec)'}, 'Color', 'm');
    ylim([-150, 150]);
    
    %Joint3 plot
    subplot(3,2,6);
    T = animatedline(etime(clock,runstart), joint3, 'Color', 'b','LineWidth',3);
    xlim([10,50]);
    ylim([-90, 90]);
    
    % Create title
    title({'Joint 3'});
    % Create xlabel
    xlabel({'Time(s)'});
    % Create ylabel
    yyaxis left
    ylabel({'Angle(degrees)'}, 'Color', 'b');
    
    yyaxis right
    Q = animatedline(etime(clock,runstart), velocity3, 'Color', 'c','LineWidth',3);
    ylabel({'Velocity(degrees/sec)'}, 'Color', 'c');
    ylim([-150, 150]);
    
    subplot(3,2,5);
    V = animatedline(etime(clock,runstart), double(xPos(4)), 'Color', [.196,.784,.235], 'LineWidth', 3);
    %Create title
    title({'X&Y Tip Position'});
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([10,50]);
    
    yyaxis left
    %Create ylabel
    ylabel({'X Position(mm)'}, 'Color', [.196,.784,.235]);
    ylim([-100,400]);
    
    yyaxis right
    W = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    %Create yylabel
    ylabel({'Z Position(mm)'}, 'Color', [.804,.216,.765]);
    ylim([-100,400]);
    traveltime = 10;
    for k = viaPts
        
        last = previous;
        
        previous = k;
        
        start = clock;
        
        joint1TrajCoef = cubicTraj(0,traveltime,0,0,last(1),k(1));
        joint2TrajCoef = cubicTraj(0,traveltime,0,0,last(2),k(2));
        joint3TrajCoef = cubicTraj(0,traveltime,0,0,last(3),k(3));
        
        pidPacket = zeros(1, 15, 'single');

        
        while(etime(clock,start)<traveltime)
            %joint 1 trajectory points
            J1 = posPoint(etime(clock,start), joint1TrajCoef(1,1), joint1TrajCoef(2,1), joint1TrajCoef(3,1), joint1TrajCoef(4,1));
            %joint 2 trajectory points
            J2 = posPoint(etime(clock,start), joint2TrajCoef(1,1), joint2TrajCoef(2,1), joint2TrajCoef(3,1), joint2TrajCoef(4,1));
            %joint 3 trajectory points
            J3 = posPoint(etime(clock,start), joint3TrajCoef(1,1), joint3TrajCoef(2,1), joint3TrajCoef(3,1), joint3TrajCoef(4,1));
            
            I = [J1;J2;J3];
            
            pidPacket(1:3) = [J1,J2,J3];
            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
            returnPacket = getStatus(pp, packet);
            curTime = etime(clock, runstart);
            
            updatePlot(fig, tip, O, P, Q, R, S, T, V, W, I, curTime, returnPacket);         

            drawnow();
        end
        
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()
