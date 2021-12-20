close all; clear; clc

%% Cam calib

K_t=1.0e+03 * [1.50    0.0000    0.640;
             0    1.50         0.512;
             0      0            0.0010];
% focal length: 0.015, pixel: 10e-6, image size: 1280x1024 (640 512);
 

% P(:, ?) = One [x;y;z] point
P = [-0.1 -0.1  0.1  0.1 -0.1 -0.1 0.1  0.1; % Matrix P represents 
     -0.1  0.1  0.1 -0.1 -0.1  0.1 0.1 -0.1; % vertice coordinates 
     -0.1 -0.1 -0.1 -0.1  0.1  0.1 0.1  0.1];% of a cube P
 
% Rot1=rotx(0)*roty(0.0)*rotz(0);
Rot1 = [1 0 0;0 1 0; 0 0 1];
tran1 = [0; 0; 1.0];

for i=1:length(P)
    Xc=[Rot1 tran1]*[P(:,i);1];
    xc= K_t * Xc;
    pt(:,i) = round([xc(1)/xc(3); xc(2)/xc(3)]);
    
    % Explain :
    % P = position in world
    % xc = intrinsic param * P
    
    % x position on image plane = fx*Px + cx * z (pixel position)
    % y position on image plane = fy*Py + cy * z (pixel position)
    
    % finally, x pos on image plane /= z
    % finally, y pos on image plane /= z
    
    % x(pixel) = fx*Px/z + cx
    % y(pixel) = fy*Py/z + cy
    
    % move x=1 in world, move x = f/z in image plane
    % move x=1 in image, move x = z/f in world.
    
end

% Rot2=rotx(0)*roty(0.2)*rotz(0);
Rot2 = [cos(0.2) 0 sin(0.2); 0 1 0; -sin(0.2) 0 cos(0.2)];
tran2= [0.2 ; 0.1; 1.3];

for i=1:length(P)
    Xc=[Rot2 tran2]*[P(:,i);1];
    xc= K_t * Xc
    pt_s(:,i)= round([xc(1)/xc(3); xc(2)/xc(3)]);
end

figure(1)
plot(pt(1,:),pt(2,:),'or','MarkerSize',10); hold on;
plot(pt_s(1,:),pt_s(2,:),'ob','MarkerSize',10); hold off;
axis([0 K_t(1,3)*2 0 K_t(2,3)*2])
grid on;