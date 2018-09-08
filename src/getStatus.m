function status = getStatus(pp, packet)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

STAT_SERV_ID = 21;

pp.write(STAT_SERV_ID, packet);

pause(.004);

status = pp.read(STAT_SERV_ID);


end
