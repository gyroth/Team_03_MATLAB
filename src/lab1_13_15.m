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
    PID_SERV_ID = 37;            % we will be talking to server ID 37 on
    % the Nucleo
    
    STAT_SERV_ID = 21;       % when passing this ID talk to the status server
    
    CALIB_SERV_ID = 25;      % when passing this ID talk to the calibration10
    % server
    
    myHIDSimplePacketComs.setVid(vid);
    
    DEBUG   = true;          % enables/disables debug prints
    needsCalibrate = true;   %needs to calibrate?
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes.
    packet = zeros(15, 1, 'single');
    
    % The following code generates a sinusoidal trajectory to be
    % executed on joint 1 of the arm and iteratively sends the list of
    % setpoints to the Nucleo firmware.
    
    viaPtsAngles = [[0; 5; 0], [-45; 20; 15], [45; 20; 15]];
    %, [0; 5; 0]
    viaPts = [[0; 0; 0],[0; 0; 0],[0; 0; 0]];
    viaPts = viaPtsAngles * 1024 / 90;
    
    % C = pid(Kp,Ki,Kd)
    
    %   disp(viaPtsAngles)
    %   disp(viaPts)
    d = size(viaPtsAngles);
    
    % The following definitions declare matrices
    test = [1,2,3,4];            % matrix to iterate through with the for loop
    
    positions = zeros(3,3,3);    % 3D matrix to store three readings from the
    % status server
    
    average = zeros(3,3);        % matrix that will hold the average of each
    % layer of the 3D matrix above
    
    home = zeros(1,3);           % matrix that will hold the new home position matrices
    
    currentPos = zeros(1,3);  % matrix to store most recent set of positions
    
    currentAngle = zeros(1,3);% matrix to store most recent set of angles
    
    %Multiple calls to pp.command do not return updated encoder readings
    % Iterate through points given
        if needsCalibrate
            disp('calibrating')
            for n = test
                tic
                %incremtal = (single(k) / sinWaveInc);
                packet = zeros(15, 1, 'single');
                packet(1) = n;
    
                %Send packet to the server and get the response
                 pp.write(STAT_SERV_ID, packet);
                 pause(.004);
                 returnPacket = pp.read(STAT_SERV_ID);
    
                % Sets the received packet into a 3x3 matrix
                returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
                    returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
                    returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
    
                if DEBUG
                    %disp('Sent Packet:');
                    %disp(packet);
                    disp('Received Packet:');
                    disp(returnPacketMatrix);
    
                end
    
    
                if n > 1
                    positions(:,:,n-1) = returnPacketMatrix;
                end
    
                toc
                pause(1) %timeit(returnPacket) !FIXME why is this needed?
    
    
            end
    
            % Averages the layers of the 3D matrix
            average = ((positions(:,:,1) + positions(:,:,2) + positions(:,:,3))/3);
            % Sets the home position as the first column of the averaged layers
            home = average(:,1);
    
            disp('Final')
            disp(positions);
            %csvwrite(['lab1Q7_' datestr(now,'mmddyyHHMMSS')  '.csv'],positions);
            disp('Average')
            disp(average)
            disp('Home')
            disp(home)
    
            % sends the new home packet to the calibration server
            pp.write(CALIB_SERV_ID,home)
    
    
            toc
            % gets the position froreturnPacketMatrixm the status server to determine whether the
            % position reflects the new home
            pause(1);
            pp.write(STAT_SERV_ID, packet);
            pause(.004)
            newHome = pp.read(STAT_SERV_ID);
    
            %converts the received packet to a matrix
            newHomeMatrix = [newHome(1,1) newHome(2,1) newHome(3,1);
                newHome(4,1) newHome(5,1) newHome(6,1);
                newHome(7,1) newHome(8,1) newHome(9,1)];
            disp('New Home')
            disp(newHomeMatrix)
    
            pause(1)
    
            needsCalibrate = false;
            disp('Calibrated')
        end
    for k = viaPts
        disp('seeking location')
        pidPacket = zeros(1, 15, 'single');
        pidPacket(1:3) = k;
        pp.write(STAT_SERV_ID, pidPacket);
        pause(.004);
        returnPacket = pp.read(STAT_SERV_ID);
        
        returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
            returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
            returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
        
        currentPos = transpose(returnPacketMatrix(:,1));
        
        %transfer encoder ticks into angles
        currentAngle = currentPos * 90 /1000;
        
        link1Line = animatedline(toc,double(currentAngle(1)),'LineWidth',3,'Color','r');
        link2Line = animatedline(toc,double(currentAngle(2)),'LineWidth',3,'Color','g');
        link3Line = animatedline(toc,double(currentAngle(3)),'LineWidth',3,'Color','b');
        drawnow
        
        while (abs(currentPos(1)-k(1))>=50 && abs(currentPos(2)-k(2))>=50 && abs(currentPos(3)-k(3))>=50)
            
            %     incremtal = (single(k) / sinWaveInc);
            packet = zeros(15, 1, 'single');
            packet(1) = 0;
            
            %     Send packet to the server and get the response
            pp.write(STAT_SERV_ID, packet);
            pause(.004);
            returnPacket = pp.read(STAT_SERV_ID);
            
            %     Sets the received packet viaPts(k)into a 3x3 matrix
            returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
                                  returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
                                  returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
            disp(returnPacketMatrix)
            
            % flips the returned matrix so that the positions are in a
            % row instead of a column
            currentPos = transpose(returnPacketMatrix(:,1));
            disp ('currpos updated')
            disp(currentPos(1))
            disp(currentPos(2))
            disp(currentPos(3))
            disp('desired')
            disp(k(1))
            disp(k(2))
            disp(k(3))
            %transfer encoder ticks into angles
            currentAngle = currentPos * 90 /1000;
            
            addpoints(link1Line,toc,double(currentAngle(1)));
            addpoints(link2Line,toc,double(currentAngle(2)));
            addpoints(link3Line,toc,double(currentAngle(3)));
            drawnow
            
            dlmwrite('lab1Q15.csv' ,currentAngle,'delimiter',',','-append');
            %     incremtal = (single(k) / sinWaveInc);
            packet = zeros(15, 1, 'single');
            packet(1) = 0;
            
            %     Send packet to the server and get the response
            pp.write(STAT_SERV_ID, packet);
            pause(.004);
            returnPacket = pp.read(STAT_SERV_ID);
            
            %     Sets the received packet viaPts(k)into a 3x3 matrix
            returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
                                  returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
                                  returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
            pidPacket = zeros(1, 15, 'single');
            pidPacket(1:3) = k;
            pp.write(PID_SERV_ID, pidPacket);
            pause(.004);
            pidReturnPacket = pp.read(PID_SERV_ID);
        end                                             %END OF WHILE LOOP
        
        disp('at location')
        disp ('currpos')
        disp(currentPos(1))
        disp(currentPos(2))
        disp(currentPos(3))
        disp('desired')
        disp(k(1))
        disp(k(2))
        disp(k(3))
        
        %disp("PID");
        %disp(pidPacket);
        %     returnAngles = zeros(20,3,'single');
        %     returnAngles = circshift(returnAngles,-1,1);
        %     returnAngles(1,1:3) = pp.command(STAT_SERV_ID, packet)*90 /1000;
        
        
        
        
        %     plot(returnAngles)
        %         for t= 0:99
        %             returnAngles = pp.command(STAT_SERV_ID, packet);
        %             returnAngles = returnAngles*90 /1000
        %             returnAnglesMatrix = [returnAngles(1,1), returnAngles(4,1), returnAngles(7,1)];
        %             dlmwrite('lab1Q13.csv' ,returnAnglesMatrix,'delimiter',',','-append');
        %             pause(.001)
        %         end
        pause(2)
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
% Clear up memory upon termination
pp.shutdown()

