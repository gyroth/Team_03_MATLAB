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
    
    %initializes variables to 0
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
    
    desP = zeros(3,1);
    
    %initialize the home position when locating centroids of object
    home = [175,0,169.28];
    
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
    viaPos = objectLoc;
    
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
    
    while(stillObjects(img))
        %state = "still Objects";
        %||||||| IMPLEMENTING STATE MACHINE HERE |||||||
        %switch(state)
        %case "still Objects"
        %   takes picture
%         %% Picture Processing and Object Location
%         % takes a picture
%         img = snapshot(cam);
%         state = "where is it";

        %case "where is it"
        %   gets x-y location
%         % processes the picture and returns the location of the first
%         % object in task space (yellow,blue,green order)
%         objectInfo = locObject(img);
%         
%         % Separates the targeted output to get the location
%         objectLoc = objectInfo(1:3);
%         state = "what color";

        %case "what color"
        %   gets the color
%           objectColor = objectInfo(4);
%           state = "go to Z above";

        %case "go to Z above"
        %   trajectory and move to X,Y of object staying at home Z
%        %current tip position and desired joint angles
%       while(~reachedSetpoint(pos(:,4),k))        
%        %% Obtaining points
%        % Waypoint in task space in millimeters to z above
%        viaPos = [objectLoc(1:2),home(3)];
%        
%        viaJtsAngles = zeros(size(viaPos));
%        
%        numPoints = size(viaJtsAngles);
%        
%        for a = 1:numPoints(2)
%            viaJtsAngles(:,a) = iKin(viaPos(:,a));
%        end
%        %ticks, ticks/s, ADC bits
%        returnPacket = getStatus(pp, packet);
%        
%        %Joint angles(deg)
%        currentAngle = processStatus(returnPacket);
%        
%        %ADC bits
%        currentTor = processStatusTor(returnPacket);
%        
%        %Joint Torque (1x3 Matrix)
%        appTorque = appliedTorque(currentTor); %%what measurement is this
%        torque in
%        
%        %Force at Tip
%        Ftip = statics3001(currentAngle', appTorque'); %%what measurement
%        is this force in
%        
%        %Where is the tip right now
%        pos= calcJointPos(currentAngle);
%        
%        %calculates the coefficients for the trajectory of the arm from
%        %"HOME" to above object (same Z plane)
%        joint1TrajCoef = quintTraj(0,traveltime,0,0,last(1),k(1),0,0);
%        joint2TrajCoef = quintTraj(0,traveltime,0,0,last(2),k(2),0,0);
%        joint3TrajCoef = quintTraj(0,traveltime,0,0,last(3),k(3),0,0);
%        
%            %joint 1 trajectory points
%            J1 = quintPoint(etime(clock,moveStart), joint1TrajCoef(1,1), joint1TrajCoef(2,1), joint1TrajCoef(3,1), joint1TrajCoef(4,1), joint1TrajCoef(5,1), joint1TrajCoef(6,1));
%            %joint 2 trajectory points
%            J2 = quintPoint(etime(clock,moveStart), joint2TrajCoef(1,1), joint2TrajCoef(2,1), joint2TrajCoef(3,1), joint2TrajCoef(4,1), joint2TrajCoef(5,1), joint2TrajCoef(6,1));
%            %joint 3 trajectory points
%            J3 = quintPoint(etime(clock,moveStart), joint3TrajCoef(1,1), joint3TrajCoef(2,1), joint3TrajCoef(3,1), joint3TrajCoef(4,1), joint3TrajCoef(5,1), joint3TrajCoef(6,1));          
            
%        pidPacket(1:3) = viaJts;
%        
%        pp.write(PID_SERV_ID, pidPacket);
%        pause(.004);
%        pidReturnPacket = pp.read(PID_SERV_ID);
%        end

        %case "gripper open?"
        %   make sure the gripper is open
        
        %case "pick it up"
        %   vertical trajectory to the object
        
        %case "weigh object"
        %   measure force at tip and determine if heavy or light
        
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