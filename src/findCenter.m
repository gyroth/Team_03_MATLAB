function [returnVal] = findCenter(color,img)
%findCenter Finds the center of the largest given spherical shape in mn locato

h = fspecial('average');

%blur filter
imgfilt = imfilter(img,h);
blurred = imfilter(imgfilt,h);

% %segment image
% 
% [segImg,~] = segmentImage(blurred);
% imshow(segImg);
% 
% for a = 1:3
% segImg(:,:,a) = blurred(:,:,a) | segImg(:,:,a);
% end
% 
% imshow(segImg);

if color == 'yellow'
    mask = yMask3(blurred);
end

if color == 'blue'
    mask = bMask3(blurred);
end

if color == 'green'
    mask = gMask3(blurred);
end

noDots = medfilt2(mask);
actuallyNoDots = medfilt2(noDots);
seriouslyNoDots = medfilt2(actuallyNoDots);
imgfilt2 = imfilter(seriouslyNoDots,h);

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
if(idx~=0)
    centroid = cat(1, c(idx).Centroid);
    
    imshow(img);  
    %disp(color);
    present = 1;
else
    present = 0;
    centroid = [0,0];
    %X = sprintf('NO %s', color);
    %disp(X);
end
returnVal = {centroid, present, color};
end

