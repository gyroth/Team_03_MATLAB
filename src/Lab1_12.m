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
clear java;
clear;
%clear import;
clear classes;
vid = hex2dec('3742');
pid = hex2dec('0007');
disp (vid );
disp (pid);
javaaddpath ../lib/SimplePacketComsJavaFat-0.5.2.jar;
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
  SERV_ID = 21;            % we will be talking to server ID 37 on
                           % the Nucleo
  STAT_SERV_ID = 21;
  
myHIDSimplePacketComs.setVid(vid);

  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  positions = zeros(3,3,3);
  angles = zeros(10,3);
  currentPos = zeros(1,3);
  currentAngle = zeros(1,3);
  anglesLog = zeros(10,4);
  timestamp = zeros(10,1);
  timeAngles = zeros(10,4);
  currTimeAngles = zeros(1,4);
  
  k=10
  
  % Iterate through a sine wave for joint values
  while true
      tic
      %incremtal = (single(k) / sinWaveInc);

       %Send packet to the server and get the response
            returnPacket = pp.command(STAT_SERV_ID, packet);
            returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
                                  returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
                                  returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
      if DEBUG
          %disp('Sent Packet:');
          %disp(packet);
          disp('Received Packet:');
          disp(returnPacketMatrix);
                     
          
      end
      
      currentPos = transpose(returnPacketMatrix(:,1));
      %transfer encoder ticks into angles
      currentAngle = currentPos * 90 /1000;
      toc
      pause(1) %timeit(returnPacket) !FIXME why is this needed?
      angles = circshift(angles,-1,1);
      angles(10,:) = currentAngle;
      timestamp = circshift(timestamp,-1,1);
      time = str2double(datestr(now,'MMSS'));
      disp(time)
      timestamp(10,1)= time;
      disp(timestamp)
      
      currTimeAngles(1,1:3) = currentAngle;
      currTimeAngles(1,4)= time;
      
      dlmwrite('lab1Q12.csv' ,currTimeAngles,'delimiter',',','-append');
      
      plot(angles)
      timeAngles (:,1:3) = angles;
      timeAngles (:,4) = timestamp;
      
      pause(.5);
     end
      
      
  

  disp('Final')
  disp(positions);
   %csvwrite(['lab1Q7_' datestr(now,'mmddyyHHMMSS')  '.csv'],positions);
   csvwrite(['lab1Q12_' datestr(now,'mmddyyHHMMSS')  '.csv'],anglesLog);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
% Clear up memory upon termination
pp.shutdown()


toc
