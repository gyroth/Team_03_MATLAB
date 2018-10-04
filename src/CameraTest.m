clc
clear
%% Instantiate hardware (turn on camera)
if ~exist('cam', 'var') % connect to webcam iff not connected
    cam = webcam();
    pause(1); % give the camera time to adjust to lighting
end

img = snapshot(cam);

while(stillObjects(img))
    img = snapshot(cam);
    
    yCen = findCenter("yellow",img);
    yLoc = yCen{1};
    bCen = findCenter("blue",img);
    bLoc = bCen{1};
    gCen = findCenter("green",img);
    gLoc = gCen{1};
    
    if yCen{2}
        ystat = 'yellow';
        
    else
        ystat = 'no yellow';
    end
    
    if bCen{2}
        bstat = 'blue';
        
    else
        bstat = 'no blue';
    end
    
    if gCen{2}
        gstat = 'green';
        
    else
        gstat = 'no green';
    end
    
    X = sprintf('There is %s, there is %s, and there is %s', ystat, bstat, gstat);
    %disp(X)
    
    hold on
        
    plot((yLoc(1)),(yLoc(2)),'r*')
    plot((gLoc(1)),(gLoc(2)),'g*')
    plot((bLoc(1)),(bLoc(2)), 'b*')
        
    hold off
    
    %X,Y,Z location of yellow in task space
    yLoc = [mn2xy((yLoc(1)),(yLoc(2))),0];
end