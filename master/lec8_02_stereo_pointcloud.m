close all; clear; clc

takeoff

%%

imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');
leftsub =rossubscriber('/stereo/left/image_raw','sensor_msgs/Image');
rightsub =rossubscriber('/stereo/right/image_raw','sensor_msgs/Image');

pause(1);

scale=1/2;

% check it by $rostopic echo /stereo/left/camera_info -> K
Intrin_left = [381.36246688113556*scale, 0.0, 320.5*scale; 0.0, 381.36246688113556*scale 240.5*scale; 0.0, 0.0, 1.0];
cameraParameters1 = cameraParameters('IntrinsicMatrix',Intrin_left);

% Check it by $rostopic echo /stereo/right/camera_info -> K 
Intrin_right = [381.36246688113556*scale, 0.0, 320.5*scale; 0.0, 381.36246688113556*scale 240.5*scale; 0.0, 0.0, 1.0];
cameraParameters2 = cameraParameters('IntrinsicMatrix',Intrin_right);

% No rotation between left & right
rotationOfCamera2=eye(3);

% Check it at stereo camera sdf file. 
% translationOfCamera2 = [-70;0;0];
translationOfCamera2 = [-200;0;0];

stereoParams = stereoParameters(cameraParameters1,cameraParameters2,rotationOfCamera2,translationOfCamera2);
    
reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];
times = [];
handles = guidata(lec8_stereo_GUIDE);

% map = occupancyMap3D(10);
obs =0.8;
kp=1; kd=0.1; ki=0.1;
Kdz=0.15; kiz=0.05;
   
for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)

    times = [times;time];
    
    xd(i) = 0;
    yd(i) = 0;
    zd(i) = 2;
    
    vxd(i) = 0;
    vyd(i) = 0;
    vzd(i) = 0;
    
    state = receive(odomsub);
    imu = receive(imusub);

    left_msg=receive(leftsub);
    right_msg=receive(rightsub);

    [lo_image,l_alpha] = readImage(left_msg);
    [ro_image,r_alpha] = readImage(right_msg);

    x=state.Pose.Pose.Position.X;
    y=state.Pose.Pose.Position.Y;
    z=state.Pose.Pose.Position.Z;
    vx=state.Twist.Twist.Linear.X;
    vy=state.Twist.Twist.Linear.Y;
    vz=state.Twist.Twist.Linear.Z;
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];

    l_image=imresize(lo_image,scale);
    r_image=imresize(ro_image,scale);

    [J1, J2] = rectifyStereoImages(l_image,r_image,stereoParams);

    disparityRange = [0 16];
    disparityMap = disparityBM(im2gray(J1),im2gray(J2),'DisparityRange',disparityRange);
    points3D = reconstructScene(disparityMap, stereoParams);
    
    % Convert to meters and create a pointCloud object
    % No use x, y from it. just use z.
    points3D = points3D(:,:,3) ./ 1000;
    
    Sd = size(l_image, [1,2]);
    [pX, pY] = meshgrid(1:Sd(2), 1:Sd(1));
    pX = pX(7:Sd(1)-7, 7:Sd(2)-7);
    pY = pY(7:Sd(1)-7, 7:Sd(2)-7);
    
    pX=pX-Intrin_left(1,3) + 0.5;
    pY=pY-Intrin_left(2,3) + 0.5;
    
    xDf = double(points3D/Intrin_left(1,1));
    yDf = double(points3D/Intrin_left(2,2));
    
    pX = pX.*xDf;
    pY = pY.*yDf;
    
    pXY=cat(3,pX,pY);
    cloud = cat(3,pXY,points3D);
    cloud = reshape(cloud, [], 3);
    pcs = rmmissing(cloud);
    
    count=1;
    
    rotmZYX = eul2rotm([-pi/2 0 -pi/2], 'ZYX');
    Rot= quat2rotm(quat) * rotmZYX;
    trans = [x; y; z];
    
    cloud_affine=zeros(size(pcs));
    parfor k=1:length(pcs)
        temp = [rotmZYX trans]*[pcs(k,:)';1];
        cloud_affine(k,:) = temp';
    end
    
    ptCloud = pointCloud(cloud_affine);

    [ptCloudOut, inlierIndices, outlierIndices] = pcdenoise(ptCloud, 'NumNeighbors', 4*16, 'Threshold', 0.1);

    plot3(ptCloudOut.Location(:,1), ptCloudOut.Location(:,2), ptCloudOut.Location(:,3), 'ok', 'MarkerSize', 1, 'parent', handles.axes1);

    hold(handles.axes1, 'off' );    grid(handles.axes1,'on');

    imshow(l_image,'Parent',handles.axes2);
    imshow(disparityMap, [0,64],'Parent',handles.axes3);

    pX=[];
    pY=[];
    pZ=[];
    
    velmsg.Twist.Linear.X = 0;
    velmsg.Twist.Linear.Y = 0;
    velmsg.Twist.Linear.Z = 2 - z;
    velmsg.Twist.Angular.Z = 0.1;
    send(velpub, velmsg);
    
    waitfor(rate);

end
 rosshutdown