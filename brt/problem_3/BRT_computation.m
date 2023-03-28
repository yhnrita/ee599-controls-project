function [safety_controller, worst_dist, data, tau, g, derivatives] = BRT_computation(params)
% unpack params
obsX1 = params.obsX1;
obsY1 = params.obsY1;
obsX2 = params.obsX2;
obsY2 = params.obsY2;
speed = params.speed;
wMax = params.wMax;
dMax = params.dMax;
width1 = params.obswidth1;
height1 = params.obsheight1;
width2 = params.obswidth2;
height2 = params.obsheight2;

%% TODO
% Define the grid for the computation: g
grid_min = [-10; -10; -pi];
grid_max = [10; 10; pi];
N = [51; 51; 51];
g = createGrid(grid_min, grid_max, N);

%% TODO
% Define the failure set: data0
obs1 = shapeRectangleByCorners(g, [obsX1, obsY1, -inf], [obsX1+width1, obsY1+height1, inf]); 
obs2 = shapeRectangleByCorners(g, [obsX2, obsY2, -inf], [obsX2+width2, obsY2+height2, inf]);
data0 = shapeUnion(obs1, obs2);
%data0 = obs1;

% time
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

% Define dynamic system
dCar = DubinsCar([0, 0, 0], wMax, speed, dMax);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% HJIextraArgs.obstacles = obs;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
derivatives = computeGradients(g, data(:,:,:,end));
safety_controller =  dCar.optCtrl([], [], derivatives, 'max');
worst_dist =  dCar.optDstb([], [], derivatives, 'min');
tau = tau2;

