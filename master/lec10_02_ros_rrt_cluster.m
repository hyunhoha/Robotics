close all; clear; clc

% addpath('/home/iasl/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')

rosshutdown
rosinit
% addpath('../gui_test')

% gui_image();
%%
global desiredRate 
desiredRate= 5;
loopTime =20;
pause(1);
rate = rosrate(desiredRate);
pause(1);
rate.OverrunAction = 'slip';

odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');

depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');

% pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');

% 2. object clustering 해보기 

K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);


for i=1:15
    setmsg.Pose.Position.X = 0;
    setmsg.Pose.Position.Y = 0;
    setmsg.Pose.Position.Z = 1;
    send(setpub,setmsg);
    pause(0.1);
       
    if rem(i,5)==0
        arming = rossvcclient('mavros/cmd/arming');
        testreq1 = rosmessage(arming);
        testreq1.Value=1;
        response1 = call(arming,testreq1,'Timeout',2);
        if testreq1.Value==1
            disp('Arming enabled');
        else
           disp('Arming failed');

        end

        set_mode = rossvcclient('mavros/set_mode');
        testreq2 = rosmessage(set_mode);
        testreq2.CustomMode='OFFBOARD';
        response2 = call(set_mode,testreq2,'Timeout',2);
        if testreq2.CustomMode=='OFFBOARD'
            disp('Offboard enabled');
        else
           disp('Offboard failed');

        end
    end

end


xf=[8;2];

xfz=1;

xd(:,1)=[0;0];
xdz=1;

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(test_img);

%% rrt initialize
flag=1;
preobs=[];
xd(:,1)=[0,0];
id=1;
map=binaryOccupancyMap(20,20,10);

%%
for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
    eul = quat2eul(quat);
    eul=[-eul(1) -eul(2) eul(3)];
    Rot=eul2rotm(eul);
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
    
    trans=[x;y;z];
    
        transxy=[x;y];

    set(handles.datax, 'String', x);
    set(handles.datay, 'String', y);
    set(handles.dataz, 'String', z);
%     set(handles.datavx, 'String', vx);
%     set(handles.datavy, 'String', vy);
%     set(handles.datavz, 'String', vz);
    
    % resize image
    scale=1/1;
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
    
    cloud_nan=cat(3,pXY,dep_mm);
    cloud_nan=reshape(cloud_nan,[],3)/1000;
%     tic
    cloud = rmmissing(cloud_nan);
%     toc
    n=length(cloud);
    cam_eul=[pi/2+pi 0 pi/2+pi];
    rotcam=eul2rotm(cam_eul);

    cloud_affine=[];
    cloud_affine=([Rot trans]*[rotcam zeros(3,1);0 0 0 1]*[cloud';ones(1,n)])';
    
    ptCloud=pointCloud(cloud_affine);
    ptCloud_d=pcdownsample(ptCloud,'gridAverage',0.1);
    
%     [groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(ptCloud_d);

%     ptCloud_obs = rmmissing(nonGroundPtCloud.Location);

    cloud_affine2 = ptCloud_d.Location;
    ptsz = size(cloud_affine2, 1);
    for i=ptsz:-1:1
        if abs(cloud_affine2(i,3) - z) > 0.4
            cloud_affine2(i,:) = [];
        end
    end
    
    ptCloud_obs = pointCloud(cloud_affine2);

    
    minDistance=0.5;
    minPoints=10;
    [labels,numClusters] = pcsegdist(ptCloud_obs,minDistance,'NumClusterPoints',minPoints);

    
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloud_obs,idxValidPoints);
    
%     figure
%     colormap(hsv(numClusters))
%     pcshow(segmentedPtCloud.Location,labelColorIndex)
%     title('Point Cloud Clusters')

    numobs=double(max(labels));
    obs=zeros(2,numobs);offset=10;
    if z>0.5 
        
        for k=1:numobs
            temp=select(segmentedPtCloud,labelColorIndex==k);
            obs(1,k)=mean(temp.Location(:,1))+offset;
            obs(2,k)=mean(temp.Location(:,2))+offset;
        end
    
       if flag==1
            map=binaryOccupancyMap(20,20,10);
            setOccupancy(map,obs',ones(numobs,1));
            inflate(map,0.8);
            flag=0;
       end
    end
    obs-offset
     plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )
    plot3(xd(1,:),xd(2,:),xfz*ones(size(xd(2,:))),'LineWidth',2,'Color', 'r','parent',handles.axes1); 
    if numClusters~=0
        plot3(segmentedPtCloud.Location(:,1),segmentedPtCloud.Location(:,2),segmentedPtCloud.Location(:,3),'ok','MarkerSize',1,'parent',handles.axes1); 
    else
         plot3(ptCloud_d.Location(:,1),ptCloud_d.Location(:,2),ptCloud_d.Location(:,3),'ok','MarkerSize',1,'parent',handles.axes1); 

    end
    
    hold( handles.axes1, 'off' );    grid(handles.axes1,'on');
    axis(handles.axes1,[-2 10 -4 4 -1 3]);
    xlabel(handles.axes1,'x');
    ylabel(handles.axes1,'y');
    zlabel(handles.axes1,'z');

    imshow(img,'Parent',handles.axes2);
    show(map,'Parent',handles.axes3);  hold( handles.axes3,'on' ); 
    
    pX=[];
    pY=[];
    pZ=[];

    setmsg.Pose.Position.X = xd(1,id);
    setmsg.Pose.Position.Y = xd(2,id);
    setmsg.Pose.Position.Z = xfz;
        
    send(setpub,setmsg);
    waitfor(rate);


end
 rosshutdown

 