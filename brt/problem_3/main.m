clear all; close all; clc;

addpath(genpath('..\helperOC'))  
addpath(genpath('..\ToolboxLS'))
addpath(genpath('..\..\'))

%% Get the params
params = get_params();

%% TODO: Reachability analysis: Please complete the template in the BRT_computation.m file for this section to get the params
disp('Pre-computing the safety controller with the BRT.........................')
[params.safety_controller, params.worst_dist, params.data, params.tau, params.g, params.derivatives] = BRT_computation(params); % the BRT gives us the safety controller for free

%% Simulate trajectory with online filtering
% The robot trajectory initialized
traj = [params.xinit];      % Trajectory history
cont_traj = [];             % Control trajectory


while ~stopping_criteria(traj(:,end),params)        % If the current state is not within the goal state

    current_state = traj(:,end);                    % Current state is the latest state in the trajectory history

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % test noise generation
    params.noise_drift = tests(current_state,params);
    disturbance = norm(params.noise_drift);         % magitude of disturbance

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    params.d = disturbance;

    % get the nominal controller: This is given to you
    u_nom = get_nominal_controller(current_state, params);   % ?????????

    switch params.controller_choice
        case 0
            cont_traj = [cont_traj;u_nom];
            chosen_controller = u_nom;
        case 1
            % TODO get the least restrictive safety filtered controller
            chosen_controller = get_safety_controller(current_state,u_nom,params);
        case 2
            % TODO get the quadratic program filtered controller
            chosen_controller = get_qpfilter_controller(current_state,u_nom,params);
    end
    
    % update the control trajectory
    if params.controller_choice > 0
        cont_traj = [cont_traj;u_nom chosen_controller];
    end
    
    % noisy update to the next state using the chosen control
    next_state = simulate(current_state, chosen_controller, 0, params.dt, params.noise_drift);
    
    traj = [traj, next_state];      % update the trajectory
    plot_env(traj,params);          % plot the env after every step
    pause(0.05)
end

% plotting the controllers
plot_controller(params.controller_choice, cont_traj);
