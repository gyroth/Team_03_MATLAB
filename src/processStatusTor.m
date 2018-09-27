function curTor = processStatusTor(returnPacket)
%Processes the output from getStatus, and returns a matrix of torques 

%First column is angles (ticks)
%Second column is velocities (ticks/sec)
%Third column is torques (ADC reading mapped between 0 and 1)
returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
    returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
    returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];

currentTor = transpose(returnPacketMatrix(:,3));

%Converts from the 0 to 1 range to raw ADC readings for a 12 bit ADC
curTor = currentTor*4095;
end
