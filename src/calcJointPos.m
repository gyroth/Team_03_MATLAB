function pos = calcJointPos(joint)
% calcJointPos:  Calculates the position of the RBE 3001 robot arm's tip with respect to the
%                   base frame
%
%               Takes in a 3x1 matrix of joint angles and plots the position of
%                   each joint and link using transform matrices
%               Returns the position vector of the end-effector

% Length of the links in mm
A = 135;
B = 175;
C = 169.28;

% Converts the given joint angles to radians
joint = deg2rad(joint);

%   D-H Param Table. 
% The following is the table containing the DH Parameters for the above
% diagram.

% a, alpha, d, theta
link1=  [0  pi/2 A joint(1)];
link2 = [B  0  0 joint(2)];
link3 = [C  0  0 joint(3)-pi/2];

% Frame 0 wrt Frame 1
T01 = dhParam(link1(1),link1(2),link1(3),link1(4));

% Frame 1 wrt Frame 2
T12 = dhParam(link2(1),link2(2),link2(3),link2(4));

% Frame 2 wrt Frame 3
T23 = dhParam(link3(1),link3(2),link3(3),link3(4));

% Frame 0 wrt Frame 2
T02 = T01 * T12;

% Frame 0 wrt Frame 3
T03 = T01 * T12 * T23;

% The positional matrix of the end-effector
%pos = [px,py,pz];

% The positions of each joints of the arm in task space in mm
armJointXPos = [0,T01(1,4),T02(1,4),T03(1,4)];
armJointYPos = [0,T01(2,4),T02(2,4),T03(2,4)];
armJointZPos = [0,T01(3,4),T02(3,4),T03(3,4)];

% Matrix of each joint position and the tip in task space
pos = [armJointXPos;
       armJointYPos;
       armJointZPos];
end



