rosshutdown
rosinit

%%
desiredRate = 20;
loopTime = 30;
pause(1);
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
[velpub,velmsg] = rospublisher('/mavros/setpoint_velocity/cmd_vel','geometry_msgs/TwistStamped');

pause(1);

disp('hello')

statesub = rossubscriber('/mavros/state', 'mavros_msgs/State'); % Drone's state from simulator
arming = rossvcclient('mavros/cmd/arming');
testreq1 = rosmessage(arming);
testreq1.Value=1;

while (1)
    response1 = call(arming,testreq1,'Timeout',2);
    st = receive(statesub); % Get Drone's State
    if st.Armed == 1
        disp('Arming enabled');
        break;
    else
       disp('Arming failed');
    end
    pause(0.1);
end

set_mode = rossvcclient('mavros/set_mode');
testreq2 = rosmessage(set_mode);
testreq2.CustomMode='OFFBOARD';

velmsg.Twist.Linear.X=0;
velmsg.Twist.Linear.Y=0;
velmsg.Twist.Linear.Z=0;


while (1)
    send(velpub,velmsg);
    response2 = call(set_mode,testreq2,'Timeout',2);
    st = receive(statesub);
    if strcmp(st.Mode, 'OFFBOARD')
        disp('Offboard enabled');
        break;
    else
       disp('Offboard failed');
    end

end