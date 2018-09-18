%%
% RBE3001 - Laboratory 3
%
% 
% ------------
% This MATLAB script is optimized to check the speed of the coonnection
% between Matlab and Nucleo
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
try
    packet = zeros(15, 1, 'single');
    i = 0;
        
    while (i<500)    
        tic
        getStatus(pp,packet);
        %dlmwrite('Lab3_Q1.csv',toc,'delimiter',',','-append');
        i = i+1; 
    end
    
    delays = csvread('Lab3_Q1.csv');

    avg= mean(delays)
    std= std(delays)
    max= max(delays)
    min= min(delays)
    
    histogram(delays)
    set(gca, 'YScale', 'log')
catch exception
    getReport(exception)
    %disp('Exited on error, clean shutdown');
end
pp.shutdown()