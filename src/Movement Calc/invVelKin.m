function [ desJointVel ] = invVelKin( curAng, desTVel )
%invVelKin: Uses the current joint angles and the instantaneous joint
%velocities to calculate the desired joint velocities

jaco = jacob0(curAng);

inverse = inv(jaco(1:3,:));

desJointVel = rad2deg(inverse) * desTVel;

end
