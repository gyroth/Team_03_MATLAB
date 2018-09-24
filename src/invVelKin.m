function [ desTaskVel ] = invVelKin( curAng, desTVel )
%fwdVelKin: Uses the current joint angles and the instantaneous joint
%velocities to calculate the desired joint velocities

jaco = jacob0(curAng);

desTaskVel = inv(jaco(1:3,:)) * desTVel;
disp(desTaskVel);

end
