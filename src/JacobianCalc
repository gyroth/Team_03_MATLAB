clear
syms theta1 A theta2 B theta3 C

vals=[theta1+90, A, 90, 0;...
    theta2, 0  , 0 , B;...
    theta3, 0  , 0 , C]

%vals=[0, 0, 90, 0;...
%    0, 0  , 0 , 0;...
%    0, 0  , 0 , 0]
for n = 1:3
    
    t = vals(n,1);
    rz= 90+t;
    tz= vals(n,2);
    rx= vals(n,3);
    tx= vals(n,4);
    
    linkvar = [rz,tz,rx,tx];
        
    if (n==1)
        transMat1= [cosd(rz), -sind(rz)*cosd(tx), sind(rz)*sind(rx), tx*(cosd(rz));...
            sind(rz), cosd(rz)*cosd(rx), -cosd(rz)*sind(rx), tx*(sind(rz));...
            0,sind(rx),cosd(rz),tz;...
            0,0,0,1];
    end
    if (n==2)
        transMat2= [cosd(rz), -sind(rz)*cosd(tx), sind(rz)*sind(rx), tx*(cosd(rz));...
            sind(rz), cosd(rz)*cosd(rx), -cosd(rz)*sind(rx), tx*(sind(rz));...
            0,sind(rx),cosd(rz),tz;...
            0,0,0,1];
    end
    if (n==3)
        transMat3= [cosd(rz), -sind(rz)*cosd(tx), sind(rz)*sind(rx), tx*(cosd(rz));...
            sind(rz), cosd(rz)*cosd(rx), -cosd(rz)*sind(rx), tx*(sind(rz));...
            0,sind(rx),cosd(rz),tz;...
            0,0,0,1];
    end
    
end

tip = transMat1*transMat2*transMat3;
tippos=[tip(1,4);tip(2,4);tip(3,4)];
tipvel = diff(tippos)