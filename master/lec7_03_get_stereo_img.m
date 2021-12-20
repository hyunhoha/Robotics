close all; clear; clc

takeoff
%%

leftsub =rossubscriber('/stereo/left/image_raw','sensor_msgs/Image');
rightsub =rossubscriber('/stereo/right/image_raw','sensor_msgs/Image');

pause(1);

K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(lec7_stereo_GUIDE);

kp=1, kd=0.1, ki=0.1
Kdz=0.15, kiz=0.05

for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    
    state = receive(odomsub);
    
    t(i)=i/10;
    xd(i)=0.05*t(i);
    yd(i)=sin(pi*t(i)/10);
    zd(i)=1;

    vxd(i)=0.05;
    vyd(i)=pi/10*cos(pi*t(i)/10);
    vzd(i)=0; 
    
    left_msg=receive(leftsub);
    right_msg=receive(rightsub);
    [left,l_alpha] = readImage(left_msg);
    [right,r_alpha] = readImage(right_msg);
    imshow(left,'Parent',handles.axes1);
    imshow(right,'Parent',handles.axes2);

    cmd.x=kp*(xd(i)-x)+kd*(vxd(i)-vx)+vxd(i);
    cmd.y=kp*(yd(i)-y)+kd*(vyd(i)-vy)+vyd(i);
    cmd.z=kp*(zd(i)-z)+kd*(vzd(i)-vz)+vzd(i);

    velmsg.Twist.Linear.X = cmd.x;
    velmsg.Twist.Linear.Y = cmd.y;
    velmsg.Twist.Linear.Z = cmd.z;
    velmsg.Twist.Angular.Z = 0.1;

    send(velpub, velmsg);
    waitfor(rate);

end
 rosshutdown