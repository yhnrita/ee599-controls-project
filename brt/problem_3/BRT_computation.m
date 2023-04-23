function [safety_controller, worst_dist, data, tau, g, derivatives] = BRT_computation(params)

% unpack params
obsX1 = params.obsX1;   % -9
obsY1 = params.obsY1;   % -2
obsZ1 = params.obsZ1;   % 0

obsX2 = params.obsX2;   % ?
obsY2 = params.obsY2;   % ?
obsZ2 = params.obsZ2;   % ?

speed = params.speed;   % 1
wMax = params.wMax;     % 1.1
dMax = params.dMax;     % ?
zMax = params.zMax;     % 1.2

length1 = params.obslength1;    % 9
width1 = params.obswidth1;      % 6
height1 = params.obsheight1;    % 6

length2 = params.obslength2;    % ?
width2 = params.obswidth2;      % ?
height2 = params.obsheight2;    % ?



% %% TODO
% % Define the grid for the computation: g
% grid_min = [-10; -10; -20];
% grid_max = [10; 10; 20];
% N = [21; 21; 21];
% g = createGrid(grid_min, grid_max, N);

% Define the grid for the computation: g
grid_min = [-10; -10; -10; -10; -10];
grid_max = [ 10;  10;  10;  10;  10];
N = [21; 21; 21; 21; 21];
g = createGrid(grid_min, grid_max, N);

%% TODO
% Define the failure set: data0
% obs1 = shapeRectangleByCorners(g, [obsX1, obsY1, -inf], [obsX1 + length1, obsY1 + width1, inf]);
% obs2 = shapeRectangleByCorners(g, [obsX2, obsY2, -inf], [obsX2 + length2, obsY2 + width2, inf]);
obs1 = shapeCuboidByCorners(g, [obsX1, obsY1, obsZ1, -inf, -inf], [obsX1 + length1, obsY1 + width1, obsZ1 + height1, inf, inf]);
% obs2 = shapeCylinder(g, [], zeros(g.dim, 1), 1);
% data0 = shapeUnion(obs1, obs2);
data0 = obs1;


%% Fixed test 2 circle
% Set the height constraint, obstacle radii, and safe step clearance epsilon
% c = 10;
% r1 = 2; % radius of the obstacle 1
% r2 = 2; % radius of the obstacle 2
% epsilon = 0.5;
% 
% % Check if the height of obstacle 1 is within the height constraint
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

%% Random test
% %obs_params.num_obs = 2;
% obs_params.max_radius = 5;
% obs_params.min_radius = 2;
% [data0, obs_info] = get_random_obstacles(g, obs_params);

%% 3D?
% ignoreDims = [1, 1, 0];
% center = [0, 0, 0];
% radius = 1;
% data0 = shapeCylinder(g, ignoreDims, center, radius);


%%

% time
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

%% Old dynamics
% Define dynamic system
% dCar = DubinsCar([0, 0, 0], wMax, speed, dMax);
dCar = DubinsCar([0, 0, 0, 0, 0], wMax, zMax, speed, dMax);

%% New dynamics
% FxRange = [-0.1, 0.1];
% FyRange = [-1, 1];
% TRange  = [-0.1, 0.1];
% %                                                   mass, inertia, dMax,    dims
% dCar = DubinsCar([0, 0, 0], FxRange, FyRange, TRange, 10,    10,      dMax);


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
HJIextraArgs.visualize.figNum = 1;              % set figure number
HJIextraArgs.visualize.deleteLastPlot = false;  % delete previous plot as you update

% uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1 0];       % plot x, y
HJIextraArgs.visualize.plotData.plotDims = [1 1 1 0 0];     % plot x, y, z
% HJIextraArgs.visualize.plotData.projpt = [0];             % project at theta = 0, zspeed = 0
HJIextraArgs.visualize.plotData.projpt = [0, 0];            % project at theta = 0, zspeed = 0
% HJIextraArgs.visualize.viewAngle = [0,90];                % view 2D
% HJIextraArgs.makeVideo = true;                            % Video Output


%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
% derivatives = computeGradients(g, data(:,:,:,end));
derivatives = computeGradients(g, data(:,:,:,:,:,end));
safety_controller =  dCar.optCtrl([], [], derivatives, 'max');
worst_dist =  dCar.optDstb([], [], derivatives, 'min');
tau = tau2;

