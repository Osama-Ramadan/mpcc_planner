clear;clc
image = imread('Maps/map1.png');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
imageOccupancy = imageOccupancy(:,:,1);
map = occupancyMap(imageOccupancy,5);
show(map)