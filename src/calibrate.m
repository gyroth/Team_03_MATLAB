function calib = calibrate(pp, packet)
%Gets the 10 position statuses from the arm, 
%then averages them and sends it to the calibration server

calculationCycles = [1,2,3,4,5,6,7,8,9,10,11];

STAT_SERV_ID = 21; % when passing this ID talk to the status server

CALIB_SERV_ID = 25;  % when passing this ID talk to the calibration

for k = calculationCycles
    
    %Send packet to the server and get the response
    returnPacket = getStatus(pp, packet);
    
    % Sets the received packet into a 3x3 matrix
    returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
        returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
        returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
    
    if k > 1
        positions(:,:,k-1) = returnPacketMatrix;
    end
    
    pause(1)
    
    
end

% Averages the layers of the 3D matrix
average = zeros(3,3,'single');

for i=1:10
average = average + positions(:,:,i);
end

average = average/10;

% Sets the home position as the first column of the averaged layers
home = average(:,1);

%appends the most recent averaged home position to a csv called lab2Q2
%dlmwrite('lab2Q2.csv' ,home,'delimiter',',','-append');

% sends the new home packet to the calibration server
pp.write(CALIB_SERV_ID,home)

% gets the position from the status server to determine whether the
% position reflects the new home
pause(1);
newHome = pp.command(STAT_SERV_ID, packet);

%converts the received packet to a matrix
calib = [newHome(1,1) newHome(2,1) newHome(3,1);
    newHome(4,1) newHome(5,1) newHome(6,1);
    newHome(7,1) newHome(8,1) newHome(9,1)];

%disp('/////////Done Calibrating\\\\\\\\\\\\\\');
end

