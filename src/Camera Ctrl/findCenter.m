function [returnVal] = findCenter(color,img)
%findCenter Finds the center of the largest given spherical shape in img

h = fspecial('average');

%blur filter
imgfilt = imfilter(img,h);
blurred = imfilter(imgfilt,h);

%Masks for the given color
if color == 'yellow'
    mask = yMask3(blurred);
end

if color == 'blue'
    mask = bMask3(blurred);
end

if color == 'green'
    mask = gMask3(blurred);
end

%Removes static noise
noDots = medfilt2(mask);
actuallyNoDots = medfilt2(noDots);
seriouslyNoDots = medfilt2(actuallyNoDots);
imgfilt2 = imfilter(seriouslyNoDots,h);

%Gets all of the centroids
c = regionprops(imgfilt2, 'centroid', 'MajorAxisLength','MinorAxisLength');
largest =  0;
idx = 0;
len = size(c);
for i = 1:len
    curMaAL = c(i).MajorAxisLength;
    curMiAL = c(i).MinorAxisLength;
    
    if(abs(curMaAL-curMiAL)<20&&curMaAL>30)
        
        if(curMaAL > largest)
            largest = curMaAL;
            idx = i;
        end
        
    end
end

%Find centroid and records its position
if(idx~=0)
    centroid = cat(1, c(idx).Centroid);
    if(centroid(1)>30 && centroid(1)<550)
    subplot(2,3,3);
    imshow(img);
    
    present = 1;
    else
        present = 0;
        centroid = [0,0];
    end
else
    present = 0;
    centroid = [0,0];
end

returnVal = {centroid, present, color};
end

