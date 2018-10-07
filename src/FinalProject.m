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

kP1 = .0001;
kI1 = .0001;
kD1 = .002;

kP2 = .004;
kI2 = 0;
kD2 = .009;

kP3 = .004;
kI3 = 0;
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
    
    xAcc = zeros(3,1,'single');
    yAcc = zeros(3,1,'single');
    zAcc = zeros(3,1,'single');
    curTime = 0;
    
    traveltime = 10;
    
    desP = zeros(3,1);
    
    %initialize the home position when locating centroids of object
    home = [190;0;30];
    
    up = [190;0;130];
    %initializes the placement positions for the weights
    %BLUE
    
    %GREEN
    
    %YELLOW
    
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
    quiv.handle = quiver3(double(xPos(4)),double(yPos(4)),double(zPos(4)),Ftip(1), Ftip(2), Ftip(3), 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScaleFactor', 15000, 'Color', 'b');
    %A.handle = plot([xPos(1,4),xPos(1,4)],[zPos(1,4),zPos(1,4)]);
    view([500,-500,500]);
    hold off
    drawnow();
    
    %time
    start = clock;
    
    %initializes the sending packet to zeros
    pidPacket = zeros(1, 15, 'single');
    gripperPacket = zeros(1,1,'single');
%     pidPacket(1)=100;
%     while(1)
%     status = gripper(pp,pidPacket)
%     end
    state = 1;
    while(stillObjects(img))
        
        switch(state)
            case 1
                % takes a picture
                img = snapshot(cam);
                state = 2;
                
            case 2 % What is the obj location
                %   gets x-y location
                % processes the picture and returns the location of the first
                % object in task space (yellow,blue,green order)
                objectInfo = locObject(img);
                
                % Separates the targeted output to get the location
                objectLoc = objectInfo(1:3);
                
                desLoc = [objectLoc{1}+175; -objectLoc{2}*.81; objectLoc{3}];
                %location above object. 175 is added to X to put object location
                %in terms of robot task space. - is added to flip Y to
                %correct side of robot task space and *.81 is to scale the mn2xy to the correct distance.
                desZLoc = [objectLoc{1}+175;-objectLoc{2}*.81; home(3)];
                
                % Waypoint to above object in task space in millimeters
                
                viaJts = xyzToTicks(desZLoc);
                
                % Waypoint to object in task space in millimeters
                
                fViaJts = xyzToTicks(desLoc);
                
                state = 3;
                
            case 3 % What is the Color
                %   gets the color
                objectColor = objectInfo(4);
                state = 4;
                
            case 4 % Move to Obj at Z point above
                %   trajectory and move to X,Y of object staying at home Z
                Ftip = moveNow(currentAngle'*1024/90,upTicks,pp,packet);
                
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %desJointAng,startPos(ticks),endPos(ticks),pp,packet
                Ftip = moveNow(currentAngle'*1024/90,viaJts,pp,packet);
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %The tip right now in task space
                aboveObjPos= calcJointPos(currentAngle);
                
                state = 5;
                
            case 5 % gripper open?
                %   open the gripper
                gripperPacket(1) = 0;
                gripper(pp,gripperPacket);
                pause(0.5);
                state = 6;
                
            case 6 % pick it up
                %   vertical trajectory to the object
                %checks current tip position and desired joint angles
                
                %desJAng,startPos,endPos,pp,packet
                Ftip = moveNow(aboveObjPos,fViaJts,pp,packet);
                
                %ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %Answers where is the tip right now
                objPos= calcJointPos(currentAngle);
                
                pause(.5);
                %CLOSE THE GRIPPER
                
                gripperPacket(1) = 1;
                gripper(pp,gripperPacket)
                
                pause(.5);
            case 7 %"weigh object"
                %   measure force at tip and determine if heavy or light
                
                %go to measure home position
                %checks current tip position and desired joint angles
                
                %desJointAng,startPos,endPos,pp,packet
                Ftip = moveNow(objPos,ticksHome,pp,packet);
                
                %ticks, ticks/s, force measure
                returnPacket = getStatus(pp,packet);
                
                %current joint torques (ADC bit)
                currentTor = processStatusTor(returnPacket);
                %current joint torques (Nmm)
                appTorque = appliedTorque(currentTor);
                
                Ftip = statics3001(appTorque);
                
                weigh = isHeavy(Ftip);
                %case "heavy"
                %   move the object to color specific point far from us outside camera bounds
                
                %case "light"
                %   move the object to color specific point close to us outside camera bounds
                
                %% Picture Processing and Object Location
                % takes a picture
                img = snapshot(cam);
                
                % processes the picture and returns the location of the first
                % object in task space (yellow,blue,green order)
                objectInfo = locObject(img);
                
                % Separates the targeted object's location and color
                objectLoc = objectInfo(1:3);
                objectColor = objectInfo(4);
                
                %% Obtaining points
                % Waypoint in task space in millimeters
                viaPos = objectLoc;
                
                viaJtsAngles = zeros(size(viaPos));
                
                numPoints = size(viaJtsAngles);
                
                for a = 1:numPoints(2)
                    viaJtsAngles(:,a) = iKin(viaPos(:,a));
                end
                
                %%ticks, ticks/s, ADC bits
                returnPacket = getStatus(pp, packet);
                
                %Joint angles(deg)
                currentAngle = processStatus(returnPacket);
                
                %ADC bits
                currentTor = processStatusTor(returnPacket);
                
                %Joint Torque (1x3 Matrix)
                appTorque = appliedTorque(currentTor);
                
                %Force at Tip
                Ftip = statics3001(currentAngle', appTorque');
                
                %Where is the tip right now
                pos= calcJointPos(currentAngle);
                
                %calculates the coefficients for the trajectory of the arm from
                %"HOME" to above object (same Z plane)
                joint1TrajCoef = quintTraj(0,traveltime,0,0,last(1),k(1),0,0);
                joint2TrajCoef = quintTraj(0,traveltime,0,0,last(2),k(2),0,0);
                joint3TrajCoef = quintTraj(0,traveltime,0,0,last(3),k(3),0,0);
                
                pidPacket(1:3) = viaJts;
                
                pp.write(PID_SERV_ID, pidPacket);
                pause(.004);
                pidReturnPacket = pp.read(PID_SERV_ID);
                
                xPos = pos(1,:);
                yPos = pos(2,:);
                zPos = pos(3,:);
                
                set(quiv.handle, 'xdata', xPos(4), 'ydata', yPos(4), 'zdata', zPos(4), 'udata', Ftip(1), 'vdata', Ftip(2), 'wdata', Ftip(3));
                
                set(P.handle,'xdata', xPos, 'ydata', yPos, 'zdata', zPos);
                
                drawnow();
                
                pause(.001);
        end
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()