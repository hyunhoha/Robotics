clear; clc

takeoff

%%
scale = 1/2;
K=([565.6008952774197*scale, 0.0, 320.5*scale; 0.0, 565.6008952774197*scale, 240.5*scale; 0.0, 0.0, 1.0]);


% load yolo trained model
pretrained = load('tinyYOLOv2-coco.mat');
detector = pretrained.yolov2Detector;

% change your rostopic image name & msg type
imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');

rgb_topic_name='/camera/rgb/image_raw';
depth_topic_name='/camera/depth/image_raw';
topic_type='sensor_msgs/Image';
sub_image=rossubscriber(rgb_topic_name,topic_type);
sub_depth=rossubscriber(depth_topic_name,topic_type);

% yolo trained image size
inputSize = pretrained.yolov2Detector.TrainingImageSize;

h=figure;

handles = guidata(lec6_GUIDE);

while ishandle(h) % if you shutdown figure window, then this code shutdown too

    image=receive(sub_image);
    depth=receive(sub_depth);
    img=readImage(image); % read rosmsg to image
    dep=readImage(depth);
    dp = imresize(dep,inputSize);
    sz=size(img);
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    x=state.Pose.Pose.Position.X;
    y=state.Pose.Pose.Position.Y;
    z=state.Pose.Pose.Position.Z;
    
    rotmZYX = eul2rotm([-pi/2 0 -pi/2], 'ZYX');
    Rot= quat2rotm(quat) * rotmZYX;
    trans=[x;y;z];
    
    if numel(img)==sz(1)*sz(2) % when image data type is grayscale(mono8 type)
        Image = cat(3, img, img, img);
        im = imresize(Image,inputSize);
    else
        im = imresize(img,inputSize);
    end
   
    cloud=[];
    dep_mm=dp*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));
    
    pX=pX-K(1,3)+0.5;
    pY=pY-K(2,3)+0.5;

    xDf=double(dep_mm/(K(1,1)));
    yDf=double(dep_mm/(K(2,2)));
    
    pX=pX.*xDf;
    pY=pY.*yDf;
    
    pXY=cat(3,pX,pY);
    
    cloud=cat(3,pXY,dep_mm);
    cloud=reshape(cloud,[],3)/1000;
    
    cloud_affine = ([Rot trans] * [cloud' ; ones([1, size(cloud,1)])])';
    
    plot3(cloud_affine(:,1),cloud_affine(:,2),cloud_affine(:,3),'ok','MarkerSize',1,'parent',handles.axes1);
    hold(handles.axes1,'on');
    
    % yolo
    [boxes,scores,labels] = detect(detector,im);
    
    %each boxes label
    if ~isempty(boxes) %prevent segmentation error
        label=char(labels);
        depth_value=lec12_Depth_extract(boxes,dp);
        score=num2str(scores);
        depth_val=num2str(depth_value);
        label_str={};
        for ii=1:size(scores)
            label_str{ii}=[label(ii,:) ' : ' score(ii,:) ', depth : ' depth_val(ii,:)];
        end
        label_to_im=label_str';
        
        im = insertObjectAnnotation(im,'rectangle',boxes,label_to_im);
        
        num_obs = size(depth_value,1);
%         for p=1:num_obs
            objxDf = double(depth_value ./ K(1,1));
            objyDf = double(depth_value ./ K(2,2));

            objX = boxes(:,1) - K(1,3) + 0.5;
            objXe = boxes(:,1) + boxes(:,3) - K(1,3) + 0.5;
            objY = boxes(:,2) - K(2,3) + 0.5;
            objYe = boxes(:,2) + boxes(:,4) - K(2,3) + 0.5;

            objedge1 = [objX.*objxDf objY.*objyDf depth_value];
            objedge2 = [objXe.*objxDf objY.*objyDf depth_value];
            objedge3 = [objXe.*objxDf objYe.*objyDf depth_value];
            objedge4 = [objX.*objxDf objYe.*objyDf depth_value];
            objedge5 = [objX.*objxDf objY.*objyDf depth_value];
            
            obj_affine = ([Rot trans] * [objedge1' objedge2' objedge3' objedge4' objedge5'; ones(1,5*num_obs)])';
            
            for y = 1:num_obs
                tmp = obj_affine(y:num_obs:4*num_obs+y,:);
                plot3(tmp(:,1), tmp(:,2), tmp(:,3), 'r-', 'MarkerSize', 50,'parent',handles.axes1);
            end
    end
    
    rotate3d(handles.axes1,'on'); grid(handles.axes1,'on'); % hold(handles.axes1,'on');
    hold(handles.axes1,'off');
    
    im = imresize(im,sz(1:2));
    imshow(im);
    
    setmsg.Pose.Position.Z = 1;
    send(setpub,setmsg);
    waitfor(rate);
 
    drawnow;
end



