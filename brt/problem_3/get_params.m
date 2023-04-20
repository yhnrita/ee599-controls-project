function params = get_params()
    %% Pack all params

    % Dynamics parameters
    params.nX = 5;                      % States - [p_x, p_y, theta]
    params.nU = 2;                      % Controls - [w] and [z]
    params.dt = 0.05;                   % Discretization step
    params.speed = 1;                   % Maximum speed - 1 m/s
    params.wMax = 1.1;                  % Maximum control - 1.1 rad/s
    params.zMax = 2.5;                  % Maximum z speed control - 1.2 m/s
    params.xinit = [-2; -5; 3; 0; 0];   % Initial state: [X] [Y] [Z] [w] [z](dot Z)

    % Setup environment parameters
    % goal params
    params.goalX = 1;
    params.goalY = 8;
    params.goalZ = 0;
    params.goalR = 0.25;

    % obstacle 1 params
    params.obsX1 = -9;
    params.obsY1 = -2;
    params.obsZ1 = -6;
    params.obslength1 = 18;    
    params.obswidth1 = 6;
    params.obsheight1 = 10;

    % obstacle 2 params
    params.obsX2 = 0;
    params.obsY2 = -2;
    params.obsZ2 = 0;
    params.obslength2 = 9;    
    params.obswidth2 = 6;
    params.obsheight2 = 6;

    % disturbance params 
    params.dMax = 0;

    % bonus: disturbance range
    params.dist_min = 0.1;
    params.dist_max = 0.8;

    % mpc params
    params.H = 10; % receeding control horizon
    
    % set controller choice: to be set by user
    params.controller_choice = 2; % choice 0 -> nominal, 1 -> least restrictive, 2 -> qp
    
    % test case: to be set by user
    params.test_choice = 0;
    
    
    % preload the optDist file for test case 4,5
    switch params.test_choice
        case 4
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_1.mat");
        case 5
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_8.mat");
    end    
end