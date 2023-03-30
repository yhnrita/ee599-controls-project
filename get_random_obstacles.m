function data = get_random_obstacles(g, params)
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
        num_obs = params.num_obs;
        max_radius = params.max_radius;
        min_radius = params.min_radius;
        max_height = params.max_height;
        min_height = params.min_height;
    catch ME
        warning(getReport(ME));
    end
    
    % Start with empty obstacles
    data0 = shapeSphere(g, [0, 0, 0], 0);
    
    for s = 1:num_obs
        rng shuffle
        c_rand = [rand(); rand(); rand()];
        center = round(g.min + (g.max - g.min) .* c_rand);
        % Obstacles should always start from the ground
        center(3) = 0;
        r = round(min_radius + (max_radius - min_radius) * rand());
        d = shapeSphere(g, center, r);
        data0 = shapeUnion(data0, d);
    end
    
    data = data0;
        