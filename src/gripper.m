function [ position ] = gripper( pp,packet)
%Controls the gripper by sending open or close messages to the NUCLEO
%firmware
GRIPPER_SERVER = 2;
pp.write(GRIPPER_SERVER, packet);
pause(.004);
position = pp.read(GRIPPER_SERVER);
pause(.004);
end

