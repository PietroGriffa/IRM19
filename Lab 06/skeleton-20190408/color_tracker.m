% tabula rasa:
clc; clear all; close all;

%% This is to solve Prelab Q1: %%

% Read the image given to you (use function imread())
img = imread('image1.jpg');

% plot the original image (use imshow())
figure(1)
imshow(img);

% convert your image into hsv color space (use function rgb2hsv())
HSV = rgb2hsv(img);

% plot the grayscale images of hue, saturation and value of your image seperately (use imshow() again)
[h s v] = imsplit(HSV);
figure(2)
montage({h, s, v, img});

% use the hue image you just plotted to find the hue lower and upper bounds for each color

% use the saturation image you just plotted and find one single lower and upper bound for all your colors

% use these tresholds to create a mask for each color, plot your three masks seperately (for each color you should have a black-white image showing only the blob of that color)
im_red = abs(img(:,:,1) - (img(:,:,2)+img(:,:,3))/2);
im_green = abs(img(:,:,2) - (img(:,:,1)+img(:,:,3))/2);
im_blue = abs(img(:,:,3) - (img(:,:,2)+img(:,:,1))/2);
imR = im2bw(im_red,treshholdR);
imG = im2bw(im_green,treshholdG);
imB = im2bw(im_blue,treshholdB);

% find the centroid of the three colors using their respective masks ( use function regionprops();  be aware that it can return more than one centroid  )
propsR = regionprops(imR,'Centroid');
propsG = regionprops(imG,'Centroid');
propsB = regionprops(imB,'Centroid');   % <-- centroid returns the corrdinate [x y]

% plot the original image with the center of the centroid (use function insertMarker())
wCentr = insertMarker(img,props.Centroid);  % <-- we will probably need a for loop, but I still don't know if we will have more centroids for the same color
figure(3)
imshow(wCentr);

