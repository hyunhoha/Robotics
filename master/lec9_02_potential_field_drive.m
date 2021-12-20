close all; clear; clc

takeoff

%%

reset(rate);
times=[];
X=[];
Y=[];
Z=[];

VX=[];
VY=[];

% Current position from /odom data is initially (0,0,0).
% It could be different from the absolute world pos.
% Obstacle position also relative to the drone.

goal = [10;0];
obstacle = [5;-0.3];

dt = 0.05;
flag = 0;
eps=0.2;

r_rho = 3.0; %influence of the obstacle
dphi_r = [0;0];

a = 10; %potential gain
K = 0.1;

% for i = 1:desiredRate*loopTime
i=1;
while(1)    
    time = rate.TotalElapsedTime;
    times=[times;time];
    fprintf('Iteration: %d - Time Elapsed: %f\n', i, time)
    
    state = receive(odomsub);
    x=state.Pose.Pose.Position.X;
    X=[X;x];
    y=state.Pose.Pose.Position.Y;
    Y=[Y;y];
    z=state.Pose.Pose.Position.Z;
    Z=[Z;z];
    
    cur_pos = [x;y];
    dphi_a = -(cur_pos - goal);
    r_obs = norm(cur_pos - obstacle);
    r_goal = norm(cur_pos - goal);
    
    n=2;
    
    if r_obs < r_rho
        dphi_r = (1 ./ ((cur_pos-obstacle)-eps) - 1/ r_rho).^2;
    else
        dphi_r = 0;
    end
    
    dphi_p(:,i) = dphi_a + dphi_r;
    dphi_p(:,i) = a*dphi_p(:,i)/(norm(dphi_p(:,i)));
    VX=[VX;dphi_p(1,i)];
    VY=[VY;dphi_p(2,i)];
    
    next_pos = cur_pos + dphi_p(:,i) * dt;
    
    if (norm(cur_pos -goal)<0.1)
        ds=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(time),'s'];
        disp(ds);
        break;
    end
    
    i = i + 1 ;
    
    setmsg.Pose.Position.X = next_pos(1);
    setmsg.Pose.Position.Y = next_pos(2);
    setmsg.Pose.Position.Z = 2;
    
    send(setpub, setmsg);
    waitfor(rate);
    
end

figure(1)
subplot(3,2,[1,2]);plot(X',Y');
hold on;
plot(obstacle(1),obstacle(2),'or','MarkerSize',10);
plot(goal(1),goal(2),'xr','MarkerSize',10);
plot(X(1),Y(1),'dr','MarkerSize',10);
hold off;
grid on;
axis([-2 12 -5 5]);
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
