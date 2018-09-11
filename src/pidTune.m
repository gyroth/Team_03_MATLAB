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

kP1 = .0001;
kI1 = .0001;
kD1 = .002;

kP2 = .004;
kI2 = 0;
kD2 = .009;

kP3 = .004;
kI3 = 0;
kD3 = .009;

firstTime = true;
try
    PID_SERV_ID = 37;   
    
    pidVal = [kP1, kI1, kD1; ...
        kP2, kI2, kD2; ...
        kP3, kI3, kD3];
    
    setPIDConstants(pp, pidVal);
    
    packet = zeros(15, 1, 'single');
    
    returnPacket = getStatus(pp, packet);
    
    % Sets the received packet into a 1x3 position matrix
    currentAngle = processStatus(returnPacket);
    
    viaPtsAngles = [[0; 5; 0], [-30; 20; 15], [30; 15 ;10] , [-10;0;54],[0;0;0]]; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  values
    viaPts = [[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0],[0; 0; 0]];
    viaPts = viaPtsAngles * 1024 / 90;
    
    %pos= calcJointPos(currentAngle);
    
    joint1 = cast(currentAngle(1),'double');
    joint2 = cast(currentAngle(2),'double');
    joint3 = cast(currentAngle(3),'double');
    
    runstart = clock;
    
    %Joint1 plot
    subplot(3,1,1);
    R = animatedline(etime(clock,runstart), joint1, 'Color', 'r','LineWidth',3);
    xlim([0, 20]);
    ylim([-60, 60]);
    
    % Create title
    title({'Joint 1 Angle'});
     % Create xlabel
    xlabel({'X-Time(ms)'});    
    % Create ylabel
    ylabel({'Y-Angle(degrees)'});
    
    %Joint2 plot
    subplot(3,1,2);
    S = animatedline(etime(clock,runstart), joint2, 'Color', 'g','LineWidth',3);
    xlim([0, 20]);
    ylim([-60, 60]);
    
    % Create title
    title({'Joint 2 Angle'});
     % Create xlabel
    xlabel({'X-Time(ms)'});    
    % Create ylabel
    ylabel({'Y-Angle(degrees)'});
    
    %Joint3 plot
    subplot(3,1,3);
    T = animatedline(etime(clock,runstart), joint3, 'Color', 'b','LineWidth',3);
    xlim([0, 20]);
    ylim([-60, 60]);
    
    % Create title
    title({'Joint 3 Angle'});
     % Create xlabel
    xlabel({'X-Time(ms)'});    
    % Create ylabel
    ylabel({'Y-Angle(degrees)'});
    
    for k = viaPts
        start = clock;
        
    pidPacket = zeros(1, 15, 'single');
            pidPacket(1:3) = k;
            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
            
        while(etime(clock,start)<3)
            
            returnPacket = getStatus(pp, packet);
            
            currentAngle = processStatus(returnPacket);
            jointAngle1 = cast(currentAngle(1), 'double');
            jointAngle2 = cast(currentAngle(2), 'double');
            jointAngle3 = cast(currentAngle(3), 'double');
            
            curTime = etime(clock, runstart);
            
            addpoints(R, curTime, jointAngle1);
            addpoints(S, curTime, jointAngle2);
            addpoints(T, curTime, jointAngle3);
            drawnow();
            
            %updatePlot(fig, returnPacket);
        end
        disp('out');
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()