%%
% RBE3001 - Laboratory 2
%
%This code draws the stick plot and updates it with the current position of the arm
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
    % Declare packet as a 15x1 matrix and fill it with zeroes
    packet = zeros(15, 1, 'single');
    
    % Calibrate the arm
    calibrate(pp, packet);
    
    % Sets the received packet into a 3x3 matrix
    returnPacket = getStatus(pp, packet);
    % Turns positions into angles
    currentAngle = processStatus(returnPacket);
    
    pos= calcJointPos(currentAngle);
    
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    fig = createStickPlot(xPos, yPos, zPos);
    
    tic
    
    %runs for 10 seconds after tic
    while(toc<10)
        returnPacket = getStatus(pp, packet);
        
        updatePlot(fig, returnPacket);
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end