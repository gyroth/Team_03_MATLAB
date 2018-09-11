function [ pos ] = posPoint( time, a_0, a_1, a_2, a_3 )
%posPoint: takes the calculated constants and time and outputs the point
%along the trajectory
%   uses the cubic polynomial trajectory function to calculate a point
%   along the trajectory

pos = a_0 + a_1*time + a_2*power(time,2) + a_3*power(time,3);

end

