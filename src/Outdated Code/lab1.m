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
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
%% The Following Code Calibrates the Arm
try
  SERV_ID = 37;            % we will be talking to server ID 37 on
                           % the Nucleo
                           
  STAT_SERV_ID = 21;       % when passing this ID talk to the status server
                           
  CALIB_SERV_ID = 25;      % when passing this ID talk to the calibration
                           % server
  
myHIDSimplePacketComs.setVid(vid);

  DEBUG   = false;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  
  % The following definitions declare matrices
  test = [1,2,3,4];            % matrix to iterate through with the for loop
  
  positions = zeros(3,3,3);    % 3D matrix to store three readings from the
                               % status server
                               
  average = zeros(3,3);        % matrix that will hold the average of each
                               % layer of the 3D matrix above
                               
  home = zeros(1,3);           % matrix that will hold the new home position matrices

  for k = test
      tic
      
      packet = zeros(15, 1, 'single');
      
      packet(1) = k;

      %Send packet to the server and get the response
            returnPacket = pp.command(STAT_SERV_ID, packet);
            
      % Sets the received packet into a 3x3 matrix
      returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
                             returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
                             returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];

     
      % Send packet to the server and get the response
      returnPacket = pp.command(SERV_ID, packet);
      toc


      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacketMatrix);
          
      end
      
            
      if k > 1 
          positions(:,:,k-1) = returnPacketMatrix;

      for x = 0:3
          packet((x*3)+1)=0.1;
          packet((x*3)+2)=0;
          packet((x*3)+3)=0;
      end
      %THis version will send the command once per call of pp.write
      pp.write(65, packet);
      pause(0.003);
      returnPacket2=  pp.read(65);
      %this version will start an auto-polling server and read back the
      %current data
      %returnPacket2=  pp.command(65, packet);
      if DEBUG
          disp('Received Packet 2:');
          disp(returnPacket2);

      end

      toc
      pause(1) %timeit(returnPacket) !FIXME why is this needed?
      
      
  end
  
  % Averages the layers of the 3D matrix
  average = ((positions(:,:,1) + positions(:,:,2) + positions(:,:,3))/3);
  
  % Sets the home position as the first column of the averaged layers
  home = average(:,1);
  
  if DEBUG
    disp('Final')
    disp(positions);
   %csvwrite(['lab1Q7_' datestr(now,'mmddyyHHMMSS')  '.csv'],positions);
    disp('Average')
    disp(average)
    disp('Home')
    disp(home)
  end
  
  % sends the new home packet to the calibration server
   pp.write(CALIB_SERV_ID,home)
 
 toc
 
   % gets the position from the status server to determine whether the
   % position reflects the new home
   pause(1);
   newHome = pp.command(STAT_SERV_ID, packet);
   
   %converts the received packet to a matrix
   newHomeMatrix = [newHome(1,1) newHome(2,1) newHome(3,1);
                    newHome(4,1) newHome(5,1) newHome(6,1);
                    newHome(7,1) newHome(8,1) newHome(9,1)];
   if DEBUG             
    disp('New Home')
    disp(newHomeMatrix)
   end
   pause(5)
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
% Clear up memory upon termination
pp.shutdown()
