close all; clear; clc

%% Cam calib

K_t=1.0e+03 * [1.50    0.0000    0.640;
             0    1.50         0.512;
             0      0            0.0010];
% focal length: 0.015, pixel: 10e-6, image size: 1280x1024 (640 512);
 
 
P = [-0.1 -0.1  0.1  0.1 -0.1 -0.1 0.1  0.1; % Matrix P represents 
     -0.1  0.1  0.1 -0.1 -0.1  0.1 0.1 -0.1; % vertice coordinates 
     -0.1 -0.1 -0.1 -0.1  0.1  0.1 0.1  0.1];% of a cube P
%  
Rot = [cos(0.2) 0 sin(0.2); 0 1 0; -sin(0.2) 0 cos(0.2)];
tran=[0.2;0.1;1.3];

for i=1:length(P)
    Xc=[Rot tran]*[P(:,i);1];
    xc= K_t * Xc;
    p3t(:,i)= round([xc(1)/xc(3);xc(2)/xc(3)]);
end


%define the 16x8 left side of Q matrix with values from matrix P
% n = 8;
n = size(P,2);

A = [ P' ones(n,1) zeros(n,4) -repmat(p3t(1,:)', 1,3).*P' ...
      zeros(n,4) P' ones(n,1) -repmat(p3t(2,:)', 1,3).*P'  ];
      
A = reshape(A',11, n*2)';
B = reshape( p3t, 1, n*2)';

Q=[A B];
if rank(Q) < 11
    error('Rank deficient,  perhaps points are coplanar or collinear');
end

[U,S,V] = svd(Q); %your code here % Singular Value Decomposition of matrix Q
C = V(:,12); %your code here
C = reshape(C,4,3)';

H=(C(:,1:3));
h=(C(:,4));

xo= inv(H)*h;%your code here

[On,Up]= qr(inv(H));%your code here
R=On.';
K=inv(Up);
K=1/K(3,3)*K*[-1 0 0;0 -1 0; 0 0 1];%rotz(pi);