function [outputArg] = iKin(inputArg)
%iKin This function takes in a 3x1 matrix of the tip position of the robot and outputs the necessary joint angles of the robot
%   Uses Inverse Kinematics to calculate the position of the joint angles for a given tip position. It will always return the up elbow
%   configuration.

%Length of Robot Arms in millimeters
L1 = 135;
L2 = 175;
L3 = 169.28;

L4 = sqrt(power(inputArg(1),2) + power(inputArg(2),2) + power((inputArg(3) - L1),2));

alpha = atan2d(inputArg(3)-L1, sqrt(power(inputArg(1),2) + power(inputArg(2),2)));
beta = acosd((power(L2,2) + power(L4,2) - power(L3,2))/(2 * L2 * L4));

%Initializes the output argument to a 3x1 vector
outputArg = zeros(3,1);

%Joint 1 Angle
outputArg(1) = atan2d(inputArg(2),inputArg(1));
jA1 = outputArg(1);

%Joint 2 Angle
outputArg(2) = alpha + beta;
jA2 = outputArg(2);

%Joint 3 Angle
outputArg(3) = -(-acosd((power(L2,2) + power(L3,2) - power(L4,2))/(2 * L2 * L3)) + 90);
jA3 = outputArg(3);
%Errors if generated angles or if the points are outside capable range of robot arm
% if or((jA1 > 90), (jA1 < -90))
%     disp(jA1);
%     error( 'Joint 1 outside bounds.')
% end
% if or((jA2 > 90), (jA2 < 0))
%     disp(jA2);
%     error( 'Joint 2 outside bounds.')
% end
% if or((jA3 > 90), (jA3< -30))
%     disp(jA3);
%     error( 'Joint 3 outside bounds.')
%end
if L4 > L2+L3
    error('Distance too far')
end
% if (power(inputArg(1),2)+power(inputArg(2),2) > (L1 + L2)*cosd(jA2))
%     disp(power(inputArg(1),2)+power(inputArg(2),2));
%     disp((L1 + L2)*cosd(jA2));
%     error( 'Distance outside possible range.')
% end
end