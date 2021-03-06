close all; clear all ; clc
%%
n=100;
x=zeros(1,n);
y=zeros(1,n);
h=zeros(1,n);
for i=1:n
    while(x(i)==y(i))
        x(i)=randi([-50 50],1,1); % separable data gneration
        y(i)=randi([-50 50],1,1);
    end
end
x=x/10;
y=y/10;

xt=-5:0.1:5;
yt=xt;
cp=0;
for i=1:n % labeling
    if (x(i)<cp*y(i))
        plot(x(i),y(i),'-ro','LineWidth',3);  % if y>x , positive
        h(i)=1;
    else
        plot(x(i),y(i),'-bp','LineWidth',3); % positive        
        h(i)=-1;
    end
    switch i
        case 1 
            hold on
        case n
            plot(xt,yt,'--k','LineWidth',2)
            hold off;axis([-5 5 -5 5]), grid on;
    end
end
%% PLA
g=zeros(1,n);

w=[0; 0; 1];
flag=1;
co=[ones(1,n);x;y];

while(1)
    for i=1:n
        g(i)=sign(w.'*co(:,i));
    end 
    index=1;
    while(index<=n)
        if(h(index)~=g(index))
            break;
        end
        index=index+1;
    end
    if (index>=n)
        break;
    end
    for i=1:3
        w(i)=w(i)+h(index)*co(i, index);
        % your code here
    end
    flag=flag+1;
end
yl=-(w(2)/w(3))*xt-w(1)/w(3);
figure,
for i=1:n
    if (x(i)<cp*y(i))
        plot(x(i),y(i),'-ro','LineWidth',3);  % if y>x , positive
    else
        plot(x(i),y(i),'-bp','LineWidth',3); % positive        
    end
    switch i
        case 1 
            hold on
        case n
            plot(xt,yt,'--k',xt,yl,'-.g','LineWidth',2);
            hold off;
            axis([-5 5 -5 5]), grid on;
    end
end
