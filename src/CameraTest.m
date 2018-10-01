%% Instantiate hardware (turn on camera)
if ~exist('cam', 'var') % connect to webcam iff not connected
    cam = webcam();
    pause(1); % give the camera time to adjust to lighting
end
h = fspecial('average');

img = snapshot(cam);
%blur filter
imgfilt = imfilter(img,h);
blurred = imfilter(img,h);

%imgfilt = imgaussfilt(img);

green = bMask3(img);
noDots = medfilt2(green);
actuallyNoDots = medfilt2(noDots);
seriouslyNoDots = medfilt2(actuallyNoDots);

imgfilt2 = imfilter(seriouslyNoDots,h);

%green2 = imfill(imgfilt2,8, 'holes');

imgedge = edge(imgfilt2);
green3 = imfill(imgedge, 'holes');




%blue = bMask2(img);
%blue2 = imfill(blue, 'holes');

%yellow = yMask2(img);
%yellow2 = imfill(yellow, 'holes');
c = regionprops(green3, 'centroid');
centroids = cat(1, c.Centroid);

imshow(green3);

hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off