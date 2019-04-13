% tabula rasa:
clc; clear all; close all;

%% This is to solve Prelab Q1: %%

% Read the image given to you (use function imread())
img = imread('image1.jpg');

% plot the original image (use imshow())
figure(1)
imshow(img);
title('Original image');

% convert your image into hsv color space (use function rgb2hsv())
HSV = rgb2hsv(img);

% plot the grayscale images of hue, saturation and value of your image seperately (use imshow() again)
[h s v] = imsplit(HSV);
figure(2)
imH = imshow(h);
title('Hue');
hp = impixelinfo(imH);  % used to find thr right treshhold values on the plot
set(hp,'Units','Normalized','Position',[0.08 0.9 0.15 0.02]);
figure(3)
imS = imshow(s);
title('Saturation');
sp = impixelinfo(imS);  % used to find thr right treshhold values on the plot
set(sp,'Units','Normalized','Position',[0.08 0.9 0.15 0.02]);
figure(4)
imV = imshow(v);
title('Value');

% use the hue image you just plotted to find the hue lower and upper bounds for each color
rh_t = [0.03    0.99];
gh_t = [0.25    0.5];
bh_t = [0.61    0.67];

% use the saturation image you just plotted and find one single lower and upper bound for all your colors
s_t = [0.89 1];

% use these tresholds to create a mask for each color, plot your three masks seperately (for each 
% color you should have a black-white image showing only the blob of that color)
hm_r = h <= rh_t(1) | h >= rh_t(2);
hm_g = h <= gh_t(2) & h >= gh_t(1);
hm_b = h <= bh_t(2) & h >= bh_t(1);
sm = s <= s_t(2) & s >= s_t(1);
mR = hm_r & sm;
mG = hm_g & sm;
mB = hm_b & sm;
figure()
imshow(mR);
title('Red');
figure()
imshow(mG);
title('Green');
figure()
imshow(mB);
title('Blue');

% find the centroid of the three colors using their respective masks ( use function regionprops();
% be aware that it can return more than one centroid  )
propsR = regionprops(mR,'Centroid');
cR = propsR.Centroid;
propsG = regionprops(mG,'Centroid');
cG = propsG.Centroid;
propsB = regionprops(mB,'Centroid');
cB = propsB.Centroid;

% plot the original image with the center of the centroid (use function insertMarker())
figure()
imgR = insertMarker(img,cR,'+','color','black','size',10); 
imgRG = insertMarker(imgR,cG,'+','color','black','size',10); 
imgRGB = insertMarker(imgRG,cB,'+','color','black','size',10); 
imshow(imgRGB);
title('Tracked locations');


