function status = getStatus(pp, packet)
%getStatus
%Returns the encoder status from the arm

STAT_SERV_ID = 21;

pp.write(STAT_SERV_ID, packet);

pause(.004);

status = pp.read(STAT_SERV_ID);


end
