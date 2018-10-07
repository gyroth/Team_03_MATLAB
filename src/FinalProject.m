%%
% RBE3001 - Final Project
%
%
% ------------
% This MATLAB script creates a live stick model plot of the robotic arm and keeps
% track of the position, velocity, and acceleration of the tip. The code
% below attempts to move the robot to the point where the user had clicked
% using ginput. Despite our best efforts, our logic seemed sound, but the
% robot did not move.
%
%
clear
clc
clear java %#ok<CLJAVA>
%clear import;
clear classes; %#ok<CLCLS>
vid = hex2dec('3742');
pid = hex2dec('0007');
%disp (vid );
%disp (pid);
javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

% Turn on Camera
if ~exist('cam', 'var') % connect to webcam iff not connected
    cam = webcam();
    pause(1); % give the camera time to adjust to lighting
end

kP1 = 0.0001;
kI1 = 0;
kD1 = 0;

kP2 = .004;
kI2 = 0;
kD2 = .009;

kP3 = .004;
kI3 = .0001;
kD3 = .009;

try
    packet = zeros(15, 1, 'single');
    
    %% Sets the PID
    PID_SERV_ID = 37;
    
    pidVal = [kP1, kI1, kD1; ...
        kP2, kI2, kD2; ...
        kP3, kI3, kD3];
    
    setPIDConstants(pp, pidVal);
    
    %% Calibrate
    returnPacket = getStatus(pp, packet);
    calibrate(pp,packet);
    
    %% Gets the Current Status
    % Sets the received packet into a 1x3 matrix of joint angles
    %ticks, ticks/s, force measure
    returnPacket = getStatus(pp,packet);
    
    %current joint angles (deg)
    currentAngle = processStatus(returnPacket);
    %current joint velocities (deg/s)
    currentVel = processStatusVel(returnPacket);
    %current joint torques (ADC bit)
    currentTor = processStatusTor(returnPacket);
    appTorque = appliedTorque(currentTor);
    
    %gets the time at the start of the program
    runstart = clock;
    
    %outputs points in task space for each coord frame
    pos= calcJointPos(currentAngle);
    
    %initializes variables
    xPos = zeros(3,1,'single');
    yPos = zeros(3,1,'single');
    zPos = zeros(3,1,'single');
    
    xVel = zeros(3,1,'single');
    yVel = zeros(3,1,'single');
    zVel = zeros(3,1,'single');
    
    curTime = 0;
    
    desP = zeros(3,1);
    
    %Ball Height(mm)
    bh = 30;
    %Camera Height(mm)
    ch = 290;
    
    %initialize the home position when locating centroids of object
    home = [190;0;30];
    
    up = [190;0;130];
    %initializes the placement positions for the weights
    %BLUE
    heavyB = [200;200;30];
    lightB = [200;-200;30];
    %GREEN
    heavyG = [175;200;30];
    lightG = [175;-200;30];
    %YELLOW
    heavyY = [150;200;30];
    lightY = [150;-200;30];
    
    ticksHome = xyzToTicks(home);
    upTicks = xyzToTicks(up);
    %% Picture Processing and Object Location
    % takes a picture
    img = snapshot(cam);
    
    % processes the picture and returns the location of the first
    % object in task space (yellow,blue,green order)
    objectInfo = locObject(img);
    
    % Separates the targeted object's location and color
    objectLoc = objectInfo(1:3);
    objectColor = objectInfo(4);
    
    
    %% Sets the position of the arm in task space
    
    % Waypoints in task space in millimeters
    viaPos = [objectLoc{1};objectLoc{2};objectLoc{3}];
    
    viaJtsAngles = zeros(size(viaPos));
    
    numPoints = size(viaJtsAngles);
    
    for a = 1:numPoints(2)
        viaJtsAngles(:,a) = iKin(viaPos(:,a));
    end
    
    % Joint Angles in Ticks at each Setpoint
    viaJts = viaJtsAngles * 1024 / 90;
    
    %Current tip velocities in mm/s
    xVelo = currentVel(1);
    yVelo = currentVel(2);
    zVelo = currentVel(3);
    
    %Current tip position in Cartesian space
    xPos = pos(1,:);
    yPos = pos(2,:);
    zPos = pos(3,:);
    
    Ftip = statics3001(currentAngle', appTorque');
    
    %Creates stickplot model in the x-z plane
    
    subplot(2,3,[1,2,4,5])
    hold on
    P = createStickPlot(xPos,yPos,zPos);
    quiv.handle = quiver3(double(xPos(4)),double(yPos(4)),double(zPos(4)),Ftip(1), Ftip(2), Ftip(3), 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScaleFactor', 15, 'Color', 'b');
    %A.handle = plot([xPos(1,4),xPos(1,4)],[zPos(1,4),zPos(1,4)]);
    view([500,-500,500]);
    hold off
    drawnow();
    
    %time
    start = clock;
    
    states = 1;
    
    %initializes the sending packet to zeros
    pidPacket = zeros(1, 15, 'single');
    gripperPacket = zeros(1,1,'single');
    %     pidPacket(1)=100;
    %     while(1)
    %     status = gripper(pp,pidPacket)
    %     end
    
    while(1)
        switch(states)
            case 1
                % takes a picture
                img = snapshot(cam);
                states = 2;
                
            case 2 % What is the obj location
                %   gets x-y location
                % processes the picture and returns the location of the first
                % object in task space (yellow,blue,green order)
                objectInfo = locObject(img);
                
                % Separates the targeted output to get the location
                objectLoc = objectInfo(1:3);
                objectLoc{1} = objectLoc{1}+25- (bh*(objectLoc{1}+30)/ch)
                
                desLoc = [objectLoc{1}+175; -objectLoc{2}*.88; objectLoc{3}]
                
                %location above object. 175 is added to X to put object location
                %in terms of robot task space. - is added to flip Y to
                %correct side of robot task space and *.81 is to scale the mn2xy to the correct distance.
                desZLoc = [objectLoc{1}+175;-objectLoc{2}*.88; home(3)];
                
                % Waypoint to above object in task space in joint ticks
                viaJts = xyzToTicks(desZLoc);
                
                % Waypoint to object in task space in joint ticks
                fViaJts = xyzToTicks(desLoc);
                
                states = 3;
                
            case 3 % What is the Color
                %   gets the color
                objectColor = objectInfo(4);
                if(objectColor{1} == "none")
                    disp("none");
                    states = 1;
                else
                    states = 4;
                end
               
                
            case 4 % Move to Obj at Z point above
                returnPacket = getStatus(pp, packet);
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %initial position in 3x1 matrix in encoder ticks
                startPosition = currentAngle'*1024/90;
                % moves to up (home) position
                Ftip = moveNow(startPosition,upTicks,pp,packet,quiv,P);
                
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %   trajectory and move to X,Y of object staying at home Z
                %desJointAng,startPos(ticks),endPos(ticks),pp,packet
                Ftip = moveNow(currentAngle'*1024/90,viaJts,pp,packet,quiv,P);
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %The tip right now in task space
                aboveObjPos= calcJointPos(currentAngle);
                
                states = 5;
                
            case 5 % gripper open?
                %   open the gripper
                gripperPacket(1) = 0;
                gripper(pp,gripperPacket);
                pause(0.5);
                states = 6;
                
            case 6 % pick it up
                %   vertical trajectory to the object
                %checks current tip position and desired joint angles
                
                currentPlace = getStatus(pp, packet);
                currentPos = processStatus(currentPlace);
                
                %desJAng,startPos,endPos,pp,packet
                Ftip = moveNow(currentPos'*1024/90,fViaJts,pp,packet,quiv,P);
                
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                
                %Joint angles(deg)
                currentAngleObj = processStatus(returnPacket);
                
                pause(.5);
                %CLOSE THE GRIPPER
                
                gripperPacket(1) = 1;
                gripper(pp,gripperPacket)
                
                pause(.5);
                states = 7;
            case 7 %"weigh object"
                %   measure force at tip and determine if heavy or light
                
                %go to measure home position
                %checks current tip position and desired joint angles
                
                %Move to home position
                %desJointAng,startPos,endPos,pp,packet
                Ftip = moveNow(currentAngleObj'*1024/90,ticksHome,pp,packet,quiv,P);
                
                pause(.5);
                
                %ticks, ticks/s, force measure
                returnPacket = getStatus(pp,packet);
                
                %current joint torques (ADC bit)
                currentTor = processStatusTor(returnPacket);
                %current joint torques (Nmm)
                appTorque = appliedTorque(currentTor);
                
                Ftip = statics3001(currentAngle',appTorque')
                
                weigh = isHeavy(Ftip);
                
                if(weigh)
                    states = 8;
                else
                    states = 9;
                end
                
            case 8 %"heavy"
                %   move the object to color specific point far from us outside camera bounds
                if objectColor{1} == "blue"
                    color = 1;
                elseif objectColor{1} == "green"
                    color = 2;
                else %yellow
                    color = 3;
                end
                
                switch color
                    case 1
                        Ftip = moveNow(ticksHome,xyzToTicks(heavyB),pp,packet,quiv,P);
                    case 2
                        Ftip = moveNow(ticksHome,xyzToTicks(heavyG),pp,packet,quiv,P);
                    case 3
                        Ftip = moveNow(ticksHome,xyzToTicks(heavyY),pp,packet,quiv,P);
                end
                gripperPacket(1) = 0;
                gripper(pp,gripperPacket);
                pause(0.5);
                states = 1;
            case 9 %"light"
                %   move the object to color specific point close to us outside camera bounds
                if objectColor{1} == "blue"
                    color = 1;
                elseif objectColor{1} == "green"
                    color = 2;
                else %yellow
                    color = 3;
                end
                
                switch color
                    case 1
                        Ftip = moveNow(ticksHome,xyzToTicks(lightB),pp,packet,quiv,P);
                    case 2
                        Ftip = moveNow(ticksHome,xyzToTicks(lightG),pp,packet,quiv,P);
                    case 3
                        Ftip = moveNow(ticksHome,xyzToTicks(lightY),pp,packet,quiv,P);
                end
                gripperPacket(1) = 0;
                gripper(pp,gripperPacket);
                pause(0.5);
                states = 1;
        end
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()