close all; clear; clc

takeoff

%%
reset(rate);
X=[];
Y=[];
Z=[];

% euler_angle = [pi/2, 0, 0];
% quat = eul2quat(euler_angle);
% 
% setmsg.Pose.Orientation.X = quat(1);
% setmsg.Pose.Orientation.Y = quat(2);
% setmsg.Pose.Orientation.Z = quat(3);
% setmsg.Pose.Orientation.W = quat(4);

loopTime = 100000;
for i = 1:desiredRate*loopTime
% while(1)

    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)

    state = receive(odomsub);
    x=state.Pose.Pose.Position.X;
    X=[X;x];
    y=state.Pose.Pose.Position.Y;
    Y=[Y;y];
    z=state.Pose.Pose.Position.Z;
    Z=[Z;z];
    
%     figure(1)
%     plot3(X,Y,Z,'-r','LineWidth',2);
%     axis([-2 2 -2 2 1 3]);
%     grid on;

    if i>desiredRate*5
%         setmsg.Pose.Position.X = 1;
%         setmsg.Pose.Position.Y = 1;
        setmsg.Pose.Position.Z = 5;
    end
%     or = set_msg.Pose.Orientation;
    
    
    send(setpub,setmsg);
    waitfor(rate);
end
 rosshutdown