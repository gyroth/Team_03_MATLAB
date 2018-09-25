function [ desJointVel ] = invVelKin( curAng, desTVel )
%fwdVelKin: Uses the current joint angles and the instantaneous joint
%velocities to calculate the desired joint velocities

disp(curAng)
jaco = jacob0(curAng)

desJointVel = inv(jaco(1:3,:)) * desTVel;


end
