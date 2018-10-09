function [ pos ] = quintPoint( time, a_0, a_1, a_2, a_3, a_4, a_5 )
%quintPoint: takes the calculated constants and time and outputs the point
%along the trajectory
%   uses the quintic polynomial trajectory function to calculate a point
%   along the trajectory

pos = a_0 + a_1*time + a_2*power(time,2) + a_3*power(time,3) + a_4*power(time,4) + a_5*power(time,5);

end
