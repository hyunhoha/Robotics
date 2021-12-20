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

% kp=0.4, kd=0.6, ki=0.1
% Kdz=0.15, kiz=0.05

kp=0.8, kd=1.5, ki=0.1
kdz=0.15, kiz=0.05

intx = 0;
inty = 0;
intz = 0;

for i = 1:desiredRate*loopTime
    
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
    
    ex = xd(i) - x;
    ey = yd(i) - y;
    ez = zd(i) - z;
    
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
    
    if z>0.3
        intx=intx+ex;
        if abs(intx)>2
            intx=sign(intx)*2;
        end
        inty=inty+ey;
        if abs(inty)>2
            inty=sign(inty)*2;
        end
        intz=intz+ez;
        if abs(intz)>1
            intz=sign(intz)*1;
        end
    end

    cmd.x=ki*intx+kp*ex+kd*(vxd(i)-vx);
    cmd.y=ki*inty+kp*ey+kd*(vyd(i)-vy);
    cmd.z=kiz*intz+kp*ez+kdz*(vzd(i)-vz);

     velmsg.Twist.Linear.X = cmd.x;
     velmsg.Twist.Linear.Y = cmd.y;
     velmsg.Twist.Linear.Z = cmd.z;
     velmsg.Header.Stamp=rostime('now');
     
    send(velpub, velmsg);
    
    waitfor(rate);
end

rotate3d(handles.axes1, 'on');
rosshutdown