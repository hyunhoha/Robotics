close all; clear; clc
%% occupancy map

%load map image and crop the size
image = imread('proj.pgm');
imageCropped = image(1:560,1:430);
% imshow(imageCropped)

imageBW = imageCropped < 100;
% imshow(imageBW)


%Create an occupany map from an example map and set map resolution as 20 cells/meter.
map = binaryOccupancyMap(imageBW,20);
show(map)

%% 

%Create a occupancyMap-based state validator using the created state space.
ss = stateSpaceSE2;

sv = validatorOccupancyMap(ss); 
sv.Map = map;

% Set validation distance for the validator.
sv.ValidationDistance = 0.01; 

ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;


% Reduce max iterations and increase max connection distance.
planner.MaxIterations = 2500;
planner.MaxConnectionDistance = 3; % set your paramter

% Set the start and goal states.
start = [3,7,0]; % set your start point
goal =  [20,25,0];% set your goal point

%Plan a path with default settings.
tic
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
toc
% visualize
map.show;
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path