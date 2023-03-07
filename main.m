clear all; close all; clc;

%% Dynamics parameters
nX = 2; % States - [p_x, p_y]
nU = 2; % Controls - [v_x, v_y]
dt = 0.05; % Discretization step
vMax = 1; % Maximum speed - 1 m/s
xinit = [1; 0]; % Initial state
noise_drift = [0; -0.01]; 

%% Setup the MPC optimization parameters
N = 150; % Optimal control horizon
H = 20; % Receding horizon

%% Setup environment parameters
goalX = 1;
goalY = 5;
obsCenterY = 3;
obsSizeY = 1;

obsCenterX = 1;
obsSizeX = 1;

% %% Call MPC function (full horizon) and propagate dynamics
% tic;
% X = mpc(N, goalX, goalY, obsCenterX, obsSizeX, obsCenterY, obsSizeY, dt, vMax, xinit);
% toc;
% [xy_full, vX, vY] = simulate_trajectory(xinit, X, N, dt, 0.0*noise_drift);
% 
% % Plot the system state
% plot(xy_full(1, :), xy_full(2, :), 'color', 'b', 'LineWidth', 2); 
% hold on; 
% xlim([-2, 4]);
% ylim([-1, 6]);
% % Create obstacle
% viscircles([obsCenterX, obsCenterY], [obsSizeX], 'color', 'r');
% % Create goal point
% viscircles([goalX, goalY], [0.1], 'color', 'g');
% % Create initial state
% viscircles(xinit', [0.1], 'color', 'b');
% plot(xy_full(1, :), xy_full(2, :), 'color', 'b', 'LineWidth', 2);

% %% Effect of noise
% [xy_full_noise, vX, vY] = simulate_trajectory(xinit, X, N, dt, noise_drift);
% % Plot the system state
% plot(xy_full(1, :), xy_full(2, :), 'color', 'b', 'LineWidth', 2); 
% hold on; 
% plot(xy_full_noise(1, :), xy_full_noise(2, :), 'color', 'b', 'linestyle', '--', 'LineWidth', 2); 
% xlim([-2, 4]);
% ylim([-1, 6]);
% % Create obstacle
% viscircles([obsCenterX, obsCenterY], [obsSizeX], 'color', 'r');
% % Create goal point
% viscircles([goalX, goalY], [0.1], 'color', 'g');
% % Create initial state
% viscircles(xinit', [0.1], 'color', 'b');

%% Call MPC function (receding horizon) and propagate dynamics
xy_receding(:, 1) = xinit;
figure,
for i=1:N
    % Solve the MPC problem
    tic;
    X = mpc(H, goalX, goalY, obsCenterX, obsSizeX, obsCenterY, obsSizeY, dt, vMax, xy_receding(:, end));
    toc;
    [xy, ~, ~] = simulate_trajectory(xy_receding(:, end), X, H, dt, noise_drift);
    xy_receding(:, i+1) = xy(:, 2);
    
    % Plot the recent trajectory 
    plot(xy(1, :), xy(2, :), 'color', 'b', 'LineWidth', 2);
    hold on,
    xlim([-2, 4]);
    ylim([-1, 6]);
    % Create obstacle
    viscircles([obsCenterX, obsCenterY], [obsSizeX], 'color', 'r');
    % Create goal point
    viscircles([goalX, goalY], [0.1], 'color', 'g');
    % Create initial state
    viscircles(xy(:, 1)', [0.1], 'color', 'b');
    % Plot the recent trajectory 
    plot(xy(1, :), xy(2, :), 'color', 'b', 'LineWidth', 2);
    hold off;
    pause(1)
end

% Plot the system state
plot(xy_receding(1, :), xy_receding(2, :), 'color', 'b', 'LineWidth', 2); 
hold on; 
xlim([-2, 4]);
ylim([-1, 6]);
% Create obstacle
viscircles([obsCenterX, obsCenterY], [obsSizeX], 'color', 'r');
% Create goal point
viscircles([goalX, goalY], [0.1], 'color', 'g');
% Create initial state
viscircles(xinit', [0.1], 'color', 'b');


