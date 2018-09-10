function emptyPacket = setPIDConstants( pp, pidVal)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
emptyPacket = 0;

PID_CONFIG_SERVER = 65;

packet = [pidVal(1,1), pidVal(1,2), pidVal(1,3), ... 
    pidVal(2,1), pidVal(2,2), pidVal(2,3), ...
    pidVal(3,1), pidVal(3,2), pidVal(3,3)];

pp.write(PID_CONFIG_SERVER, packet);

end

