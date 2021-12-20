close all; clear; clc

takeoff

%%

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];
times=[];
% ts=[];

handles = guidata(lec5_GUIDE);

loopTime = 10000;

path = [0.5000    0.5000         3
    0.6416    1.6867    3
    0.9728    2.8906    3
    1.0115    2.8889    3
    1.6580    3.4625    3
    2.3647    3.8554   3
    2.6277    4.4749   3
    2.9715    5.3539   3
    3.7834    4.9919   3
    4.2369    4.7721   3
    4.9213    4.6333   3
    5.5586    4.7275   3
    6.0842    4.5648   -1.3318
    6.7407    4.8359   -2.0330
    6.7894    5.4394   -2.3903
    6.7204    6.2517    2.9723
    6.4191    7.4086    2.9239
    6.3628    7.8915    3.1189
    6.4522    8.2287    1.4151
    6.0148    8.5208    0.0359
    5.3891    8.5448   -0.0499
    5.1472    8.7093   -1.4397
    4.4391    8.9000   -2.1125
    3.9907    8.8778   -2.2596
    3.5068    8.8100   -2.1190
    2.9358    8.6924   -1.6830
    2.7164    8.2129   -2.4218
    0.6832    9.0930   -1.8638
    0.6004   12.0800   -1.0212
    0.5175   15.0669   -0.1786
    0.1137   15.4744   -0.5162];

j = 1;

for i = 1:desiredRate*loopTime
    
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    times = [times;time];
    
    xd(i) = path(j,1) - 8;
    yd(i) = path(j,2) - 8;
    zd(i) = 3;
    
%     vxd(i) = 0.1;
%     vyd(i) = pi/10*cos(pi*time/10);
%     vzd(i) = 0;

    state = receive(odomsub);
    x=state.Pose.Pose.Position.X - 10;
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
    
    if sqrt((xd(i) - x)^2 + (yd(i) - y)^2) < 0.2
        j = j + 1;
        if j > size(path)
            break;
        end
    end
    
    % Compare desired position with current position
%     figure(1)
%     subplot(3,2,1); plot(times,X,'-b',times,xd,'-r','LineWidth',2);
%     subplot(3,2,3); plot(times,Y,'-b',times,yd,'-r','LineWidth',2);
%     subplot(3,2,5); plot(times,Z,'-b',times,zd,'-r','LineWidth',2);
%     subplot(3,2,2); plot(times,VX,'-b',times,vxd,'-r','LineWidth',2);
%     subplot(3,2,4); plot(times,VY,'-b',times,vyd,'-r','LineWidth',2);
%     subplot(3,2,6); plot(times,VZ,'-b',times,vzd,'-r','LineWidth',2);

    % Using guide, plot 3D trajectories. if too slow, remove this.
    plot3(handles.axes1, X, Y, Z, '-b', 'LineWidth', 2);
    hold(handles.axes1, 'on');
    plot3(handles.axes1, xd, yd, zd, '-r', 'LineWidth', 2);
    rotate3d(handles.axes1, 'on');
    hold(handles.axes1, 'off');
    grid(handles.axes1, 'on');
    axis(handles.axes1,[-8 8 -8 8 1 4]);
    
    set(handles.txt1, 'String', 'Current_X');
    set(handles.txt2, 'String', 'X_in');
    set(handles.txt3, 'String', x);
    set(handles.txt4, 'String', xd(i));
    
    % send set position commands.
    setmsg.Pose.Position.X = xd(i)+10;
    setmsg.Pose.Position.Y = yd(i);
    setmsg.Pose.Position.Z = zd(i);
    
    send(setpub,setmsg);

    waitfor(rate);
end
rosshutdown