close all; clear; clc

takeoff
%%

reset(rate);
X=[];
Y=[];
Z=[];
% t=[];
VX=[];
VY=[];
VZ=[];
times=[];
ts=[];

handles = guidata(lec5_GUIDE);

kp=1; kd=0.1; ki=0.1;
Kdz=0.15; kiz=0.05;

for i = 1:desiredRate*loopTime
    
%     t = i/10;
%     ts = [ts,t];
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    times = [times;time];
    
    xd(i) = 0.1*time;
    yd(i) = sin(pi*time/10);
    zd(i) = 1;
    
    vxd(i) = 0.1;
    vyd(i) = pi/10*cos(pi*time/10);
    vzd(i) = 0;
    
    state = receive(odomsub);
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
    vz=-state.Twist.Twist.Linear.Z;
    VZ=[VZ;vz];
    
    % Compare desired position with current position
    figure(1)
    subplot(3,2,1); plot(times,X,'-b',times,xd,'-r','LineWidth',2);
    subplot(3,2,3); plot(times,Y,'-b',times,yd,'-r','LineWidth',2);
    subplot(3,2,5); plot(times,Z,'-b',times,zd,'-r','LineWidth',2);
    subplot(3,2,2); plot(times,VX,'-b',times,vxd,'-r','LineWidth',2);
    subplot(3,2,4); plot(times,VY,'-b',times,vyd,'-r','LineWidth',2);
    subplot(3,2,6); plot(times,VZ,'-b',times,vzd,'-r','LineWidth',2);
    
    % Using guide, plot 3D trajectories. if too slow, remove this.
    plot3(handles.axes1, X, Y, Z, '-b', 'LineWidth', 2);
    hold(handles.axes1, 'on');
    plot3(handles.axes1, xd, yd, zd, '-r', 'LineWidth', 2);
    
    hold(handles.axes1, 'off');
    grid(handles.axes1, 'on');
    axis(handles.axes1,[-1 4 -3 3 0 3]);

    set(handles.txt1, 'String', 'Current_X');
    set(handles.txt2, 'String', 'X_in');
    set(handles.txt3, 'String', x);
    set(handles.txt4, 'String', xd(i));
    
    cmd.x=kp*(xd(i)-x)+kd*(vxd(i)-vx)+vxd(i);
    cmd.y=kp*(yd(i)-y)+kd*(vyd(i)-vy)+vyd(i);
    cmd.z=kp*(zd(i)-z)+kd*(vzd(i)-vz)+vzd(i);

    velmsg.Twist.Linear.X = cmd.x;
    velmsg.Twist.Linear.Y = cmd.y;
    velmsg.Twist.Linear.Z = cmd.z;

    send(velpub, velmsg);
    
    waitfor(rate);
end
rotate3d(handles.axes1, 'on');
rosshutdown