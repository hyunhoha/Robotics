close all; clear; clc

takeoff

%%

% Code explained
% Depth image -> get point cloud -> apply drone's translation and rotation
% to point cloud.

imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');
depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');

pause(1);
K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);

reset(rate);


handles = guidata(lec6_GUIDE);

kp=1; kd=0.1; ki=0.1;
Kdz=0.15; kiz=0.05;

max_loop = desiredRate * loopTime;

X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];
t=[];


xd = zeros(max_loop);
yd = zeros(max_loop);
zd = zeros(max_loop);
vxd = zeros(max_loop);
vyd = zeros(max_loop);
vzd = zeros(max_loop);

for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    
    image_msg=receive(imagesub);
    depth_msg=receive(depthsub);

    [img,im_alpha] = readImage(image_msg);
    [dep,dp_alpha] = readImage(depth_msg);

    x=state.Pose.Pose.Position.X;
    X=[X;x];
    y=state.Pose.Pose.Position.Y;
    Y=[Y;y];
    z=state.Pose.Pose.Position.Z;
    Z=[Z;z];
    vx=state.Twist.Twist.Linear.X;
    VX=[VX;vx];
    vy=state.Twist.Twist.Linear.Y;
    VY=[VY;vy];
    vz=state.Twist.Twist.Linear.Z;
    VZ=[VZ;vz];
    
    % translation matrix and rotation matrix
    rotmZYX = eul2rotm([-pi/2 0 -pi/2], 'ZYX');
    Rot= quat2rotm(quat) * rotmZYX;
    trans=[x;y;z];
    
    t(i)=time;
    xd(i)=0.05*t(i);
    yd(i)=sin(pi*t(i)/10);
    zd(i)=1;

    vxd(i)=0.05;
    vyd(i)=pi/10*cos(pi*t(i)/10);
    vzd(i)=0; 
     
    set(handles.datax, 'String', x);
    set(handles.datay, 'String', y);
    set(handles.dataz, 'String', z);
    set(handles.datavelx, 'String', vx);
    set(handles.datavely, 'String', vy);
    set(handles.datavelz, 'String', vz);
    
    % resize image
    scale=1/4;
    re_img=imresize(img,scale);
    re_dep=imresize(dep,scale);

    cloud=[];
    dep_mm=re_dep*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));
    
    pX=pX-K(1,3)*scale+0.5;
    pY=pY-K(2,3)*scale+0.5;

    xDf=double(dep_mm/(K(1,1)*scale));
    yDf=double(dep_mm/(K(2,2)*scale));
    
    pX=pX.*xDf;
    pY=pY.*yDf;
    
    pXY=cat(3,pX,pY);
    
    cloud=cat(3,pXY,dep_mm);
    cloud=reshape(cloud,[],3)/1000;
    cloud_affine=zeros(size(cloud));
%     tic
    parfor k=1:length(cloud_affine)
        temp=[Rot trans]*[cloud(k,:)';1];
        cloud_affine(k,:)=temp';
    end
%     toc
    
    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )
    plot3(xd(1:i),yd(1:i),zd(1:i),'LineWidth',2,'Color', 'r','parent',handles.axes1); 
    plot3(cloud_affine(:,1),cloud_affine(:,2),cloud_affine(:,3),'ok','MarkerSize',1,'parent',handles.axes1);
    
    hold( handles.axes1, 'off' );    grid(handles.axes1,'on');
%     axis(handles.axes1,[-5 15 -10 10 -3 3]);

    imshow(img,'Parent',handles.axes2);
    imagesc(dep,'Parent',handles.axes3);
    
    % point cloud initialize
    pX=[];
    pY=[];
    pZ=[];
    
    % from lect5. PD control
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