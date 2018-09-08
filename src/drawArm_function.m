function pos = drawArm_function(joint)
%fwkin: Calculates the position of the RBE 3001 robot arm's tip with respect to the
%           base frame
%
%       Takes in a 3x1 matrix of joint angles and plots the position of
%           each joint and link using transform matrices
%       Returns the position vector of the end-effector

%These matrices transform F0 to F1
    
    %The Z rotational matrix of Link 1
RotZL1 = [cosd(joint(1)) -sind(joint(1)) 0 0;
          sind(joint(1)) cosd(joint(1)) 0 0;
          0 0 1 0;
          0 0 0 1];

    %The translational matrix of Link 1
TransZL1 = [1 0 0 0;
            0 1 0 0;
            0 0 1 135;
            0 0 0 1];
        
    %The X rotational matrix of Link 1
RotXL1 = [1 0 0 0;
          0 cosd(-90) -sind(-90) 0;
          0 sind(-90) cosd(-90) 0;
          0 0 0 1];

%Frame 0 wrt Frame 1
T01 = RotZL1 * TransZL1 * RotXL1;


%These matrices transform F1 to F2

    %The translational matrix of Link 2
TransXL2 = [ 1 0 0 175;
             0 1 0 0;
             0 0 1 0;
             0 0 0 1];
         
    %The rotational matrix of Link 2
RotZL2 = [cosd(-joint(2)) -sind(-joint(2)) 0 0;
          sind(-joint(2)) cosd(-joint(2)) 0 0;
          0 0 1 0;
          0 0 0 1];
      
RotL2 = RotZL2;

%Frame 1 wrt Frame 2
T12 = RotL2 * TransXL2;

%Frame 0 wrt Frame 2
T02 = T01*T12;


%These matrices transform F2 to F3

    %The translational matrix of Link 3
TransXL3 = [1 0 0 169.28;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

    %The rotational matrix of Link 3
RotZL3 = [cosd(90 - joint(3)) -sind(90 - joint(3)) 0 0;
          sind(90 - joint(3)) cosd(90 - joint(3)) 0 0;
          0 0 1 0;
          0 0 0 1];

RotL3 = RotZL3;

%Frame 2 wrt Frame 3
T23 = RotL3 * TransXL3;

%Frame 0 wrt Frame 3
T03 = T01 * T12 * T23;

px = T03(1,4);
py = T03(2,4);
pz = T03(3,4);

%The positional matrix of the end-effector
pos = [px,py,pz];

%The positions of each joints of the arm
armJointXPos = [0,T01(1,4),T02(1,4),T03(1,4)];
armJointYPos = [0,T01(2,4),T02(2,4),T03(2,4)];
armJointZPos = [0,T02(3,4),T02(3,4),T03(3,4)];

createArmFigure(armJointXPos,armJointYPos,armJointZPos);
end

