function curAngle = processStatus(returnPacket)
%Processes the output from getStatus, and returns a matrix of angles

returnPacketMatrix = [returnPacket(1,1) returnPacket(2,1) returnPacket(3,1);
    returnPacket(4,1) returnPacket(5,1) returnPacket(6,1);
    returnPacket(7,1) returnPacket(8,1) returnPacket(9,1)];
currentPos = transpose(returnPacketMatrix(:,1));
curAngle = currentPos * 90 /1000;

end

