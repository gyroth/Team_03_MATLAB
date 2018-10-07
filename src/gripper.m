function [ position ] = gripper( pp,packet)
%gripper Summary of this function goes here
%   Detailed explanation goes here
GRIPPER_SERVER = 2;
pp.write(GRIPPER_SERVER, packet);
pause(.004);
position = pp.read(GRIPPER_SERVER);
pause(.004);
end

