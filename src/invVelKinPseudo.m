function [ desJointVel ] = invVelKinPseudo( curAng,desTVel )
%invVelKinPseudo Uses Pseudoinverse to get the inverse of J and then
%outputs the 
%   Detailed explanation goes here
j = jacob0(curAng)

jT = j'

intermediate = j*jT

jI = inv(intermediate)

desJointVel = rad2deg(jT*desTVel);

end

