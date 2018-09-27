function [ transf ] = dhParam( a, alpha, d, theta )
%dhParam The homogenous transform based off of the given DH parameters 
%   Calculates the numerical matrix for the homogenous transform using the
%   given DH parameters

transf = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
          sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),             d;
          0,           0,                       0,                       1];
end

