function [ object ] = stillObjects(img)
%stillObjects Checks to see if there are still relevant objects in the task
% space
%   Detailed explanation goes here

yCen = findCenter("yellow",img);
isY = yCen{2};
bCen = findCenter("blue",img);
isB = bCen{2};
gCen = findCenter("green",img);
isG = gCen{2};

if(isY||isB||isG)
object = 1;
else
object = 0;    
end

end

