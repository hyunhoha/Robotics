close all; clear; clc

addpath('/home/iasl/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')
% rosinit
addpath('../gui_test')
gui_image();
%%
desiredRate = 2;
loopTime =40;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
leftsub =rossubscriber('/stereo/left/image_raw','sensor_msgs/Image');
rightsub =rossubscriber('/stereo/right/image_raw','sensor_msgs/Image');

pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');

scale=1/1;
Intrin_left=[238.3515418007097*scale, 0.0, 200.5*scale; 0.0, 238.3515418007097*scale, 200.5*scale; 0.0, 0.0, 1.0]';
cameraParameters1 = cameraParameters('IntrinsicMatrix',Intrin_left);
Intrin_right=[238.3515418007097*scale, 0.0, 200.5*scale; 0.0, 238.3515418007097*scale, 200.5*scale; 0.0, 0.0, 1.0]';
cameraParameters2 = cameraParameters('IntrinsicMatrix',Intrin_right);
rotationOfCamera2=eye(3);
translationOfCamera2=[-70;0;0];
stereoParams = stereoParameters(cameraParameters1,cameraParameters2,rotationOfCamera2,translationOfCamera2);

        arming = rossvcclient('mavros/cmd/arming');
        testreq1 = rosmessage(arming);
        testreq1.Value=1;
        response1 = call(arming,testreq1,'Timeout',2);
        if testreq1.Value==1
            disp('Arming enabled');
        else
           disp('Arming failed');

        end

for i=1:20
    setmsg.Pose.Position.X = 0;
    setmsg.Pose.Position.Y = 0;
    setmsg.Pose.Position.Z = 1;
    send(setpub,setmsg);
    pause(0.1);
       
    if rem(i,5)==0

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

for i = 1:desiredRate*loopTime
    t(i)=i/desiredRate;
   xd(i)=0.05*t(i);
   yd(i)=sin(pi*t(i)/10);
   zd(i)=1;
   
   vxd(i)=0.05;
   vyd(i)=pi/10*cos(pi*t(i)/10);
   vzd(i)=0; 
   
end

    
reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(gui_image);
   
for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)

    state = receive(odomsub);
    left_msg=receive(leftsub);
    right_msg=receive(rightsub);

    [lo_image,l_alpha] = readImage(left_msg);
    [ro_image,r_alpha] = readImage(right_msg);

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

    set(handles.datax, 'String', x);
    set(handles.datay, 'String', y);
    set(handles.dataz, 'String', z);
    set(handles.datavx, 'String', vx);
    set(handles.datavy, 'String', vy);
    set(handles.datavz, 'String', vz);
    

    l_image=imresize(lo_image,scale);
    r_image=imresize(ro_image,scale);

        
    [J1, J2] = rectifyStereoImages(l_image,r_image,stereoParams);


    disparityRange = [0 16];
    
    disparityMap = disparityBM(im2gray(J1),im2gray(J2),'DisparityRange',disparityRange);
    
    points3D = reconstructScene(disparityMap, stereoParams);
    % Convert to meters and create a pointCloud object
    points3D = points3D ./ 1000;
    count=1;
    
    for k=1:size(points3D,1)
        for j=1:size(points3D,2)
            if points3D(k,j,3)<15 &&  points3D(k,j,3)>1
                pX(count) = points3D(k,j,3);
                pY(count) = -points3D(k,j,1);
                pZ(count) = -points3D(k,j,2);
                count=count+1;
            end
        end
    end
    obs=0.8;
    points_filtered=[pX' pY' pZ'];

    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )
    plot3(xd(1:i),yd(1:i),zd(1:i),'LineWidth',2,'Color', 'r','parent',handles.axes1); 
    plot3(pX,pY,pZ,'ok','MarkerSize',1,'parent',handles.axes1); 
%     show(map,'parent',handles.axes1); 
    hold( handles.axes1, 'off' );    grid(handles.axes1,'on');
    axis(handles.axes1,[-10 10 -10 10 -2 3]);

    imshow(l_image,'Parent',handles.axes2);
    imshow((disparityMap),'Parent',handles.axes3);
%     imshow(r_image,'Parent',handles.axes3);

    A = stereoAnaglyph(J1,J2);
    figure(2)
    imshow(A)

    pX=[];
    pY=[];
    pZ=[];

        setmsg.Pose.Position.X = xd(i);
        setmsg.Pose.Position.Y = yd(i);
        setmsg.Pose.Position.Z = zd(i);
        
    send(setpub,setmsg);
    waitfor(rate);


end
 rosshutdown