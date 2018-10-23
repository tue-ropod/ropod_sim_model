clear all;clc;close all;

A = ones(11);

imshow(A,'InitialMagnification',2000)

while 1
    B = uint8(ginput(1));
    A(B(2),B(1)) = 0;
    imshow(A,'InitialMagnification',2000)
    save A.mat A
end

%load A.mat
%imwrite(A,'tue_maze.png')