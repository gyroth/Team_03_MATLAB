function [ curTaskVel ] = fwdVelKin( curAng, curJVel )
%fwdVelKin: Uses the current joint angles and the instantaneous joint
%velocities to calculate the current taks space velocities

curTaskVel = jacob0(curAng)*transpose(curJVel);

end

