clear; close all; clc
%%
X=zeros(5,5,5);

X(:,:,1)=[0 1 1 0 0;
          0 0 1 0 0;
          0 0 1 0 0;
          0 0 1 0 0;
          0 1 1 1 0];
X(:,:,2)=[1 1 1 1 0;
          0 0 0 0 1;
          0 1 1 1 0;
          1 0 0 0 0;
          1 1 1 1 1];
X(:,:,3)=[1 1 1 1 0;
          0 0 0 0 1;
          0 1 1 1 0;
          0 0 0 0 1;
          1 1 1 1 0];

X(:,:,4)=[0 0 0 1 0;
          0 0 1 1 0;
          0 1 0 1 0;
          1 1 1 1 1;
          0 0 0 1 0];
      
X(:,:,5)=[1 1 1 1 1;
          1 0 0 0 0;
          1 1 1 1 0;
          0 0 0 0 1;
          1 1 1 1 0];
D=eye(5);
      
Num_input=25;
Num_hidden1=20;
Num_hidden2=20;
Num_hidden3=20;
Num_output=5;

W1=2*rand(Num_hidden1,Num_input)-1; % initial weight
W2=2*rand(Num_hidden2,Num_hidden1)-1; % initial weight
W3=2*rand(Num_hidden3,Num_hidden2)-1; % initial weight
W4=2*rand(Num_output,Num_hidden3)-1; % initial weight

maxIter=1000;
iter=1:maxIter;
N=5;
for epoch=1:maxIter
    [W1,W2,W3,W4]=Deepdropout(W1,W2,W3,W4,X,D);

end

for k=1:N % check
    x=reshape(X(:,:,k),25,1);

    % backprop error
    v1=W1*x;
    y1=Relu(v1);

    v2=W2*y1;
    y2=Relu(v2);

    v3=W3*y2;
    y3=Relu(v3);

    v=W4*y3;
    y=Softmax(v)
end

function [W1, W2,W3,W4]=Deepdropout(W1,W2,W3,W4,X,D)
    
    alpha=0.001;
    beta=0.5;
    N=5; % size
    for k=1:N
        x=reshape(X(:,:,k),25,1);
        d=D(k,:)';
        
        v1=W1*x; % first
        y1=Relu(v1);
        y1=y1.*Dropout(y1,0.2);

        v2=W2*y1; % second
        y2=Relu(v2);
        y2=y2.*Dropout(y2,0.2);

        v3=W3*y2; % third
        y3=Relu(v3);
        y3=y3.*Dropout(y3,0.2);
        
        v=W4*y3; % third
        y=Softmax(v);
       
        e=d-y;
        delta=e; % Cross entrophy
        
        disp(e)
        e3=W4'*delta;
%         delta3= e3; % code here
        delta3 = (v3>0).*e3;

        e2=W3'*delta3;
        delta2 = (v2>0).*e2;
%         disp(size(e2));
%         delta2=0;
%         disp(e2)
%         delta2 = zeros(size(e2));
%         for i=1:size(e2, 1)
%             if e2(i) >= 0
%                 delta2(i) = 1;
%             else
%                 delta2(i) = 0;
%             end
%         end
%         if e2 >= 0
%             delta2= e2; % code here
%         else
%             delta2 = 0;
%         end
% %         delta2 = e2;
        e1=W2'*delta2;
        delta1 = (v1>0).*e1;
%         delta1 = zeros(size(e1));
%         for i=1:size(e1, 1)
%             if e1(i, 1) >= 0
%                 delta1(i) = 1;
%             else
%                 delta1(i) = 0;
%             end
%         end
%         delta1 = e1;
        %         if e1 >= 0
%             delta1= e1;% code here
%         else
%             delta1 = 0;
%         end
%         disp(e1)
%         disp(size(e1))
%         disp(delta2)
        
        dW4=alpha*delta*y3'; %delta rule
        W4=W4+dW4;
        
        dW3=alpha*delta3*y2'; %delta rule
        W3=W3+dW3;
        
        dW2=alpha*delta2*y1'; %delta rule
        W2=W2+dW2;
        
%         disp(x)
        dW1=alpha*delta1*x'; %delta rule
        W1=W1+dW1;
        
    end
end

function y=Relu(x)
    y=max(0,x);
end

function ym=Dropout(y,ratio)
    [m,n]=size(y);
    ym=zeros(m,n);
    
    num=round(m*n*(1-ratio));
    idx=randperm(m*n,num); 
    ym(idx)=1/(1-ratio);
end

function y=Softmax(x)
    ex=exp(x);
    y=ex/sum(ex);
end