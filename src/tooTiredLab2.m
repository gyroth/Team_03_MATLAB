%%
% RBE3001 - Laboratory 1
%
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communicrm,	namely	it	will	make	it	swing	back	and	forth	two	times.ation between this script and the Nucleo firmware, send
%+ setpoint commands and receive sensor data.
%
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.
clear
clear java
%clear import;
clear classes;
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
    %% Sets the Waypoints of the arm in angles
    viaPtsAngles = [[0; 0; 0], [0; 15; 45], [0; 45; -10] , [0;0;0]];
    viaPts = [[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0]];
    viaPts = viaPtsAngles * 1024 / 90;
    
    %First Position in Joint Space
    joint2TrajCoef1 = cubicTraj(0,2,0,0,0,15);
    joint3TrajCoef1 = cubicTraj(0,2,0,0,0,45);
    
    %Joint 2's First Trajectory
    posPoint(0, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint(.2, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint(.4, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint(.6, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint(.8, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 1, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 1.2, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 1.4, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 1.6, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 1.8, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    posPoint( 2, joint2TrajCoef1(1,1), joint2TrajCoef1(2,1), joint2TrajCoef1(3,1), joint2TrajCoef1(4,1));
    
    %Joint 3's First Trajectory
    posPoint( 0, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( .2, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( .4, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( .6, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( .8, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 1, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 1.2, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 1.4, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 1.6, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 1.8, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    posPoint( 2, joint3TrajCoef1(1,1), joint3TrajCoef1(2,1), joint3TrajCoef1(3,1), joint3TrajCoef1(4,1));
    
    %Second Position in Joint Space
    joint2TrajCoef2 = cubicTraj(2,4,0,0,15,45);
    joint3TrajCoef2 = cubicTraj(2,4,0,0,45,-10);
    
    posPoint(0,joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint(.2, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint(.4, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint(.6, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint(.8, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 1, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 1.2, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 1.4, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 1.6, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 1.8, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    posPoint( 2, joint2TrajCoef2(1,1), joint2TrajCoef2(2,1), joint2TrajCoef2(3,1), joint2TrajCoef2(4,1));
    
    posPoint( 0, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( .2, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( .4, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( .6, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( .8, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 1, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 1.2, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 1.4, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 1.6, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 1.8, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    posPoint( 2, joint3TrajCoef2(1,1), joint3TrajCoef2(2,1), joint3TrajCoef2(3,1), joint3TrajCoef2(4,1));
    
    %Third Position in Joint Space
    joint2TrajCoef3 = cubicTraj(4,6,0,0,45,0);
    joint3TrajCoef3 = cubicTraj(4,6,0,0,-10,0);
    
    posPoint(0,joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint(.2, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint(.4, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint(.6, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint(.8, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 1, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 1.2, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 1.4, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 1.6, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 1.8, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    posPoint( 2, joint2TrajCoef3(1,1), joint2TrajCoef3(2,1), joint2TrajCoef3(3,1), joint2TrajCoef3(4,1));
    
    posPoint( 0, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( .2, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( .4, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( .6, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( .8, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 1, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 1.2, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 1.4, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 1.6, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 1.8, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    posPoint( 2, joint3TrajCoef3(1,1), joint3TrajCoef3(2,1), joint3TrajCoef3(3,1), joint3TrajCoef3(4,1));
    
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
    
    runstart = clock;
    
    %Joint1 plot
    subplot(3,2,2);
    R = animatedline(etime(clock,runstart), joint1, 'Color', 'r','LineWidth',3);
    xlim([0, 15]);
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
    subplot(3,2,4);
    S = animatedline(etime(clock,runstart), joint2, 'Color', 'g','LineWidth',3);
    xlim([0, 15]);
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
    xlim([0, 15]);
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
    title(['X&Y Tip Position']);
    
    %Create xlabel
    xlabel({'Time(s)'});
    %Limit X
    xlim([0, 15]);
    
    yyaxis left
    %Create ylabel
    ylabel({'X Position(mm)'}, 'Color', [.196,.784,.235]);
    ylim([-100,400]);
    
    yyaxis right
    W = animatedline(etime(clock,runstart), double(zPos(4)), 'Color', [.804,.216,.765], 'LineWidth', 3);
    %Create yylabel
    ylabel({'Z Position(mm)'}, 'Color', [.804,.216,.765]);
    ylim([-100,400]);
    
    for k = viaPts
        start = clock;
        
        pidPacket = zeros(1, 15, 'single');
        pidPacket(1:3) = k;
        pp.write(PID_SERV_ID, pidPacket);
        pause(.004);
        pidReturnPacket = pp.read(PID_SERV_ID);
        
        while(etime(clock,start)<3)
            returnPacket = getStatus(pp, packet);
            curTime = etime(clock, runstart);
            
            updatePlot(fig, tip, O, P, Q, R, S, T, V, W, curTime, returnPacket);

            drawnow();
        end
        
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()