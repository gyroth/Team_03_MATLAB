function J = jacob0( jAngles )
%jacob0 Calculates the Jacobian of the arm
%   Uses the current joint angles to find the Jacobain for the current
%   position
jAngles = deg2rad(jAngles);

A = 135;
B = 175;
C = 169.28;

J = [ -sin(jAngles(1))*(C*sin(jAngles(2) + jAngles(3)) + B*cos(jAngles(2))),  cos(jAngles(1))*(C*cos(jAngles(2) + jAngles(3)) - B*sin(jAngles(2))),  C*cos(jAngles(1))*cos(jAngles(2) + jAngles(3));
 -cos(jAngles(1))*(C*sin(jAngles(2) + jAngles(3)) + B*cos(jAngles(2))), -sin(jAngles(1))*(C*cos(jAngles(2) + jAngles(3)) - B*sin(jAngles(2))), -C*sin(jAngles(1))*cos(jAngles(2) + jAngles(3));
                                                0,               C*sin(jAngles(2) + jAngles(3)) + B*cos(jAngles(2)),             C*sin(jAngles(2) + jAngles(3));
                                                 0,                                       -sin(jAngles(1)),                      -sin(jAngles(1));
                                                0,                                       -cos(jAngles(1)),                      -cos(jAngles(1));
                                                 1,                                                 0,                                0];
 
 
end