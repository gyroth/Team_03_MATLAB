function [ velVec ] = calcVelVec( start,goal,vel )
%calcVelVec Calculates the Velocity Vector
%   calculates the velocity vector using the start and end positions, and
%   the desired velocity

    vector = goal - start;
    
    uVec = vector/sqrt((power(vector(1),2)+power(vector(2),2)+power(vector(3),2)));
    
    velVec = uVec*vel;
end

