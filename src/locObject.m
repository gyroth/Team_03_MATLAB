function objectPos = locObject(img)
%locObject Returns the location of a prioritized object. Color order of
% yellow, blue, green
%   Determines the location of the object with respect to the robot's
%   reference frame

xyPos = zeros(1,3);

yCen = findCenter("yellow",img);
yLoc = yCen{1};
bCen = findCenter("blue",img);
bLoc = bCen{1};
gCen = findCenter("green",img);
gLoc = gCen{1};

if(yCen{2})
    xyPos = {10*mn2xy(yLoc(1), yLoc(2)),yCen{3}};
else
    if(bCen{2})
        xyPos = {10*mn2xy(bLoc(1), bLoc(2)),bCen{3}};
    else
        if(gCen{2})
            xyPos = {10*mn2xy(gLoc(1), gLoc(2)),gCen{3}};
        end
    end
end
xyLoc = xyPos{1}
xyPos
objectPos = {xyLoc(1); xyLoc(2); 0; xyPos{2}};
end
