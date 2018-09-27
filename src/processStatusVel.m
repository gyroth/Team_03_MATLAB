function curVel = processStatusVel(returnPacket)
%Processes the output from getStatus, and returns a matrix of velocities

%First column is angles (ticks)
%Second column is velocities (ticks/sec)
%Third column is torques (ADC bits)
returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
    returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
    returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
currentVel = transpose(returnPacketMatrix(:,2));
curVel = currentVel * 90 /1024;

end