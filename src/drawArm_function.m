clear
a0=0;
a1=0;
a2=0;
angles = [a0,a1,a2];
x=fwkin(angles);
disp(x(1))
disp(x(2))
disp(x(3))
function pos = fwkin(joint)
%fwkin: Calculates the position of the RBE 3001 robot arm's tip with respect to the
%           base frame
%
%       Detailed explanation goes here

%These matrices transform F0 to F1
RotZL1 = [cosd(joint(1)) -sind(joint(1)) 0 0;
          sind(joint(1)) cosd(joint(1)) 0 0;
          0 0 1 0;
          0 0 0 1];

TransZL1 = [1 0 0 0;
            0 1 0 0;
            0 0 1 135;
            0 0 0 1];

RotXL1 = [1 0 0 0;
          0 cosd(-90) -sind(-90) 0;
          0 sind(-90) cosd(-90) 0;
          0 0 0 1];


T01 = RotZL1 * TransZL1 * RotXL1;
disp('RZ')
disp(RotZL1)
disp('RX')
disp(RotXL1)
disp('T01')
disp(T01)

%These matrices transform F1 to F2
TransXL2 = [ 1 0 0 175;
             0 1 0 0;
             0 0 1 0;
             0 0 0 1];
RotZL2 = [cosd(-joint(2)) -sind(-joint(2)) 0 0;
          sind(-joint(2)) cosd(-joint(2)) 0 0;
          0 0 1 0;
          0 0 0 1];
RotL2 = RotZL2;

T12 = RotL2 * TransXL2;
disp('T12')
disp(T12)
T02 = T01*T12;
disp('T02')
disp(T02)

%These matrices transform F2 to F3
TransXL3 = [1 0 0 169.28;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
RotZL3 = [cosd(90 - joint(3)) -sind(90 - joint(3)) 0 0;
          sind(90 - joint(3)) cosd(90 - joint(3)) 0 0;
          0 0 1 0;
          0 0 0 1];

RotL3 = RotZL3;

T23 = RotL3 * TransXL3;
disp('T23')
disp(T23)

T03 = T01 * T12 * T23;
disp('T03')
disp(T03)

px = T03(1,4);
py = T03(2,4);
pz = T03(3,4);

pos = [px,py,pz];

end

