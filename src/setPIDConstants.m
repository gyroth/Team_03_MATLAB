function emptyPacket = setPIDConstants( pp, pidVal)
%setPIDConstants transforms the given matrix of PID values into a form to be sent to the Nucleo firmware

emptyPacket = 0;

PID_CONFIG_SERVER = 65;

packet = [pidVal(1,1), pidVal(1,2), pidVal(1,3), ... 
    pidVal(2,1), pidVal(2,2), pidVal(2,3), ...
    pidVal(3,1), pidVal(3,2), pidVal(3,3)];

pp.write(PID_CONFIG_SERVER, packet);

pid = zeros(15,1);
pid=pp.read(PID_CONFIG_SERVER);

end

