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
    PID_SERV_ID = 37;            % we will be talking to server ID 37 on
    % the Nucleo
    
    STAT_SERV_ID = 21;       % when passing this ID talk to the status server
    
    CALIB_SERV_ID = 25;      % when passing this ID talk to the calibration
    % server
    
    packet = zeros(15, 1, 'single');
    
    packet(1) = 0;
    
    %Send packet to the server and get the response
    pp.write(STAT_SERV_ID, packet);
    
    pause(.004);
    
    returnPacket = pp.read(STAT_SERV_ID);
    
    
    % Sets the received packet into a 3x3 matrix
    returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
        returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
        returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
    currentPos = transpose(returnPacketMatrix(:,1));
    currentAngle = currentPos * 90 /1000;
        
    pos= drawArm_function(currentAngle)
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    R.handle= plot3(xPos,yPos,zPos,'DisplayName','Robot Links','MarkerFaceColor',[1 0 0],...
    'MarkerEdgeColor',[0 0 0],...
    'MarkerSize',10,...
    'Marker','square',...
    'LineWidth',4,...
    'Color',[0.850980401039124 0.325490206480026 0.0980392172932625]);

    xlim([-200, 350]);
    ylim([-350, 350]);
    zlim([-75, 500]);
    
    
    % Create xlabel
    xlabel({'X-Distance(mm)'});
    
    % Create zlabel
    zlabel({'Z-Distance(mm)'});
    
    % Create title
    title({'Robotic Arm Stickplot'});
    
    % Create ylabel
    ylabel({'Y-Distance(mm)'});
    
    while(1)
        %Send packet to the server and get the response
        pp.write(STAT_SERV_ID, packet);
        
        pause(.004);
        
        returnPacket = pp.read(STAT_SERV_ID);
        
        % Sets the received packet into a 3x3 matrix
        returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
            returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
            returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
        currentPos = transpose(returnPacketMatrix(:,1));
        currentAngle = currentPos * 90 /1000
        
        pos= drawArm_function(currentAngle);
        xPos = pos(1,:)
        yPos = pos(2,:)
        zPos = pos(3,:)
        
        set(R.handle, 'xdata', xPos, 'ydata', yPos, 'zdata', zPos);
        drawnow();
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end