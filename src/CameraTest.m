clc
clear
%% Instantiate hardware (turn on camera)
if ~exist('cam', 'var') % connect to webcam iff not connected
    cam = webcam();
    pause(1); % give the camera time to adjust to lighting
end

while(1)
    img = snapshot(cam);
    
    yCen = findCenter("yellow",img);
    bCen = findCenter("blue",img);
    gCen = findCenter("green",img);
    
    img = snapshot(cam);
    imshow(img);
    
    if yCen(3) == 'true'
        ystat = 'yellow';
        
    else
        ystat = 'no yellow';
    end
    
    if bCen(3) == 'true'
        bstat = 'blue';
        
    else
        bstat = 'no blue';
    end
    
    if gCen(3) == 'true'
        gstat = 'green';
        
    else
        gstat = 'no green';
    end
    
    X = sprintf('There is %s, there is %s, and there is %s', ystat, bstat, gstat);
    %disp(X)
    
    hold on
        
    plot(str2double(yCen(1)),str2double(yCen(2)),'r*')
    plot(str2double(gCen(1)),str2double(gCen(2)),'g*')
    plot(str2double(bCen(1)),str2double(bCen(2)), 'b*')
        
    hold off
    
    %X,Y,Z location of yellow in task space
    yLoc = [mn2xy(str2double(yCen(1)),str2double(yCen(2))),-34]
end