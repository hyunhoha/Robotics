close all; clear; clc

takeoff

%%

imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');
depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');

scale = 1/2;
K=([565.6008952774197*scale, 0.0, 320.5*scale; 0.0, 565.6008952774197*scale, 240.5*scale; 0.0, 0.0, 1.0]);

xd=0;
yd=0;
zd=1;

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(lec9_GUIDE);


goal = [10;0];
dt = 0.05;
flag = 0;
eps = 0.2;

r_rho = 5.0;
dphi_r = [0;0];

a=10;
times=[];
obs_means = [];

for i = 1:desiredRate*loopTime

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    times = [times;time];
    
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    rotmZYX = eul2rotm([-pi/2 0 -pi/2], 'ZYX');
    Rot = quat2rotm(quat) * rotmZYX;
    
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
    
%     set(handles.datax, 'String', x);
%     set(handles.datay, 'String', y);
%     set(handles.dataz, 'String', z);
%     set(handles.datavelx, 'String', vx);
%     set(handles.datavely, 'String', vy);
%     set(handles.datavelz, 'String', vz);
    
    % resize image
    re_img=imresize(img,scale);
    re_dep=imresize(dep,scale);

    cloud=[];
    dep_mm=re_dep*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));
    
    pX=pX-K(1,3)+0.5;
    pY=pY-K(2,3)+0.5;

    xDf=double(dep_mm/(K(1,1)));
    yDf=double(dep_mm/(K(2,2)));
    
    pX=pX.*xDf;
    pY=pY.*yDf;
    
    pXY=cat(3,pX,pY);
    
    cloud_nan=cat(3,pXY,dep_mm);
    cloud_nan=reshape(cloud_nan,[],3)/1000;
    cloud = rmmissing(cloud_nan);
    n=length(cloud);

    cloud_affine=[];
    cloud_affine=([Rot trans]*[cloud';ones(1,n)])';
    
    ptCloud=pointCloud(cloud_affine);
    ptCloud_d=pcdownsample(ptCloud,'gridAverage',0.1);
    
    cloud_affine2 = ptCloud_d.Location;
    ptsz = size(cloud_affine2, 1);
    for i=ptsz:-1:1
        if abs(cloud_affine2(i,3) - z) > 0.4
            cloud_affine2(i,:) = [];
        end
    end
    
    ptCloud_obs = pointCloud(cloud_affine2);
    
    minDistance = 2;
    minPoints = 20;
    [labels,numClusters] = pcsegdist(ptCloud_obs,minDistance,'NumClusterPoints',minPoints);    
    idxValidPoints = find(labels);
    segmentedPtCloud = select(ptCloud_obs,idxValidPoints);
   
    label_nums = size(unique(labels));

    obstacles = [double(labels) ptCloud_obs.Location];

    for k=1:label_nums
        obs_lb = obstacles(:,1) == k;            
        obs_means(k, :) = [sum(obstacles(:,2).*obs_lb)./sum(obs_lb) sum(obstacles(:,3).*obs_lb)./sum(obs_lb) sum(obstacles(:,4).*obs_lb)./sum(obs_lb)];
    end    

%     hold(handles.axes1,'on');
%     plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )

    plot(X,Y,'LineWidth', 2, 'color', 'b', 'parent',handles.axes1);
    hold(handles.axes1,'on');
%     colors = ["ob", "or", "ok", "oy", "og"];
    if size(obs_means,1) ~= 0
        plot(obs_means(:,1), obs_means(:,2), 'ok', 'parent', handles.axes1);
    end 
%     for w=1:size(labels,1)
%         plot3(obstacles(w,2), obstacles(w,3), obstacles(w,4), colors(obstacles(w,1)+1) , 'MarkerSize',1,'parent',handles.axes1);
%     end
            
    hold( handles.axes1, 'off' );    grid(handles.axes1,'on');
%     axis(handles.axes1,[-2 10 -4 4 -1 3]);
%     xlabel(handles.axes1,'x');
%     ylabel(handles.axes1,'y');
%     zlabel(handles.axes1,'z');
    
    imshow(img,'Parent',handles.axes2);
    imagesc(dep,'Parent',handles.axes3);
    
    pX=[];
    pY=[];
    pZ=[];
    
%     K=0.1;
    
%     i=1;
    cur_pos = [x;y];
    dphi_a = -(cur_pos - goal);
    
    r_goal = norm(cur_pos - goal);
    r_obs = [];
    for q=1:label_nums
        r_obs(q) = norm(cur_pos - obs_means(q));
        if r_obs(q) < r_rho
            dphi_r = 0.5 * dphi_r + 0.5 * (1 ./ ((cur_pos-obs_means(q))-eps) - 1/ r_rho).^2;
        end
    end
    dphi_p = dphi_a + dphi_r;
    dphi_p = a*dphi_p / (norm(dphi_p));
%     VX = [VX;dphi_p(1)];
%     VY = [VY;dphi_p(2)];
    
    next_pos = cur_pos + dphi_p(:,i) * dt;
    
    if (norm(cur_pos - goal)<0.5)
        ds=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(time),'s'];
        disp(ds);
        break;
    end
    
    setmsg.Pose.Position.X = next_pos(1);
    setmsg.Pose.Position.Y = next_pos(2);
    setmsg.Pose.Position.Z = 2;
    
    send(setpub, setmsg);
    waitfor(rate);
    
end
    
rosshutdown
    
figure(1)
subplot(3,2,[1,2]);plot(X',Y');
hold on;
% plot(obs_means(:,1),obs_means(:,2),'or','MarkerSize',10);
plot(goal(1),goal(2),'xr','MarkerSize',10);
plot(X(1),Y(1),'dr','MarkerSize',10);
hold off;
grid on;
axis([-2 12 -1 1]);
xlabel('x');
ylabel('y');
subplot(3,2,3); plot(times,X,'-b');
grid on;
ylabel('x')
subplot(3,2,4);plot(times,Y,'-b');
grid on;
ylabel('y')
subplot(3,2,5); plot(times,VX,'-b');
grid on;
ylabel('vx')
xlabel('times')
subplot(3,2,6);plot(times,VY,'-b');
grid on;
ylabel('vy');
xlabel('times')

    
    

    