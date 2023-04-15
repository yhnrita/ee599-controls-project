function [data, obs_info] = get_random_obstacles(g, params)
    %% Input params = {num_obs, max_radius, min_radius, max_height, min_height}
    if nargin < 1
        % Require a grid input
        error("Require a grid input")
    end
    
    rng shuffle
    % Create 1-4 obstacles
    num_obs = round(1 + (4-1)*rand());
    max_radius = 3;
    min_radius = 1;
    max_height = 15;
    min_height = 1;
    
    % If user specified input params, use those instead
    try
        if isfield(params, num_obs)
            num_obs = params.num_obs;
        end
        if isfield(params, max_radius)
            max_radius = params.max_radius;
        end
        if isfield(params, min_radius)
            min_radius = params.min_radius;
        end
        if isfield(params, max_height)
            max_height = params.max_height;
        end
        if isfield(params, min_height)
            min_height = params.min_height;
        end
    catch ME
        warning(getReport(ME));
    end
    
    % Start with empty obstacles
    data0 = shapeCylinder(g, [], [0, 0, 0], 0);
    obs_info = {};
    
    for s = 1:num_obs
        rng shuffle
        c_rand = [rand(); rand(); rand()];
        center = round(g.min + (g.max - g.min) .* c_rand);
        % Obstacles should always start from the ground
        center(3) = 0;
        r = round(min_radius + (max_radius - min_radius) * rand());
        %d = shapeSphere(g, center, r);
        d = shapeCylinder(g, [3], center, r);
        data0 = shapeUnion(data0, d);
        
        % Mark the obstacle info in the format of [center_x, center_y, r]
        obs_info{s} = [center(1), center(2), r];
    end
    %data0 = shapeCylinder(g, [3], [0.1, 0.2, 0], 0.5);
    
    data = data0;
        