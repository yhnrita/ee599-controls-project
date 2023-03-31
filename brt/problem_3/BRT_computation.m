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
grid_min = [-10; -10; -20];
grid_max = [10; 10; 20];
N = [51; 51; 51];
g = createGrid(grid_min, grid_max, N);

%% TODO
% % Define the failure set: data0
% obs1 = shapeRectangleByCorners(g, [obsX1, obsY1, -inf], [obsX1+width1, obsY1+height1, inf]); 
% obs2 = shapeRectangleByCorners(g, [obsX2, obsY2, -inf], [obsX2+width2, obsY2+height2, inf]);
% data0 = shapeUnion(obs1, obs2);
% %data0 = obs1;

%% TODO
%% Define the failure set: data0
% Set the height constraint, obstacle radii, and safe step clearance epsilon
c = 10;
r1 = 2; % radius of the obstacle 1
r2 = 2; % radius of the obstacle 2
epsilon = 0.5;

% Check if the height of obstacle 1 is within the height constraint
% if height1 <= c
%     % Create an outer sphere for obstacle 1 with a radius increased by epsilon
%     obs1_outer = shapeSphere(g, [obsX1, obsY1, 0], r1+epsilon);
%     % Create an inner sphere for obstacle 1 with a radius decreased by epsilon
%     obs1_inner = shapeSphere(g, [obsX1, obsY1, 0], r1-epsilon);
%     % Calculate the difference between the outer and inner spheres for obstacle 1
%     obs1 = shapeDifference(obs1_outer, obs1_inner);
% else
%     % If the height constraint is not met, create a sphere for obstacle 1 with radius r1
%     obs1 = shapeSphere(g, [obsX1, obsY1, 0], r1+epsilon);
% end
% 
% % Check if the height of obstacle 2 is within the height constraint
% if height2 <= c
%     % Create an outer sphere for obstacle 2 with a radius increased by epsilon
%     obs2_outer = shapeSphere(g, [obsX2, obsY2, 0], r2+epsilon);
%     % Create an inner sphere for obstacle 2 with a radius decreased by epsilon
%     obs2_inner = shapeSphere(g, [obsX2, obsY2, 0], r2-epsilon);
%     % Calculate the difference between the outer and inner spheres for obstacle 2
%     obs2 = shapeDifference(obs2_outer, obs2_inner);
% else
%     % If the height constraint is not met, create a sphere for obstacle 2 with radius r2
%     obs2 = shapeSphere(g, [obsX2, obsY2, 0], r2+epsilon);
% end
% 
% % Combine the two obstacle representations into a single data set
% data0 = shapeUnion(obs1, obs2);

%obs_params.num_obs = 2;
obs_params.max_radius = 5;
obs_params.min_radius = 2;
data0 = get_random_obstacles(g, obs_params);
% 
% ignoreDims = [1, 1, 0];
% center = [0, 0, 0];
% radius = 1;
% data0 = shapeCylinder(g, ignoreDims, center, radius);

% time
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'min';
dMode = 'min';

% Define dynamic system
controlRange = [-wMax, wMax];
dCar = DubinsCar([0, 0, 0], controlRange, controlRange, controlRange, 1, speed, dMax);
%x, FxRange, FyRange, TRange, mass, inertia, dMax, dims

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

