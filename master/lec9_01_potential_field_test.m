close all; clear; clc
%%

xd=[12;1];
x(:,1)=[0;6];
vx(:,1)=[0;0];

bx=6; % obstacle position x
by=4;  % obstacle position y
obs=[bx;by];

dt=0.05;
t(1)=0;
ns=size(t,2);

flag=1;
i=1;
eps=0.2;

r_rho=1.0; % infludence of the obstacle
dphi_r=[0;0];

a=2; % potential gain
k=0.1;

while(flag)
    % dphi_a = goal - current_pos (== straight line)
    % repulsive potential.. (pulling force)
    dphi_a=-(x(:,i)-xd);
    
    % r_obs : the distance between current_pos and the obstacles.
    r_obs=norm(x(:,i)-obs);
    
    % r_goal : the distance between current_pos and the goal.
    r_goal=norm(x(:,i)-xd);
    
    n=2;
    
    % if the distance from obstacle is in range,
    if r_obs<r_rho
        
        % push potential
        dphi_r= -(1 / (r_obs) - 1/ r_rho)^2;
          
          % new dphi_r (= initially [0;0])
%           dphi_r= -k*(1/(r_obs-eps)-1/r_rho)*(x(:,i)-obs)/r_obs/(r_obs-eps)^2*(r_goal)^2+k*(1/(r_obs-eps)-1/r_rho)^2*(dphi_a);

        %your code here
    else
         dphi_r=0;
    end

    % total potential (attract to goal, avoid from obstacle)
    dphi_p(:,i)=dphi_a+dphi_r;
    
    % a == potential gain. (Unit vector * gain)
    dphi_p(:,i)=a*dphi_p(:,i)/(norm(dphi_p(:,i)));
    
    % ndphi == norm of that. total force.. == a
    ndphi(i)=(norm(dphi_p(:,i)));
    
    % velocity == attraction
    vx(:,i+1)=(dphi_p(:,i));
    
    % position update
    x(:,i+1)=x(:,i)+(dphi_p(:,i))*dt;
    
    % time update
    t(i+1)=t(i)+dt;
    
    % arrival check
    if (norm(x(:,i+1)-xd)<0.1)
    	break;
        X=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(t(i+1)),'s'];
        disp(X);
    end

    i=i+1;
end
% i
    figure(1)
    subplot(3,2,[1,2]);plot(x(1,:),x(2,:));
    hold on;
    plot(obs(1),obs(2),'or','MarkerSize',10);
    plot(xd(1),xd(2),'xr','MarkerSize',10);
    plot(x(1,1),x(2,1),'dr','MarkerSize',10);
    hold off;
    grid on;
    axis([0 12 0 8]);
   	xlabel('x');
    ylabel('y');
    subplot(3,2,3); plot(t,x(1,:),'-b');
    grid on;
    ylabel('x')
    subplot(3,2,4);plot(t,x(2,:),'-b');
    grid on;
    ylabel('y')
    subplot(3,2,5); plot(t,vx(1,:),'-b');
    grid on;
    ylabel('vx')
    xlabel('times')
    subplot(3,2,6);plot(t,vx(2,:),'-b');
    grid on;
    ylabel('vy');
    xlabel('times')
