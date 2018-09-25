function [ velVec ] = calcVelVec( start,goal,vel )
%calcVelVec Summary of this function goes here
%   Detailed explanation goes here

    vector = goal - start;
    
    uVec = vector/sqrt((power(vector(1),2)+power(vector(2),2)+power(vector(3),2)));
    
    velVec = uVec*vel;
end

