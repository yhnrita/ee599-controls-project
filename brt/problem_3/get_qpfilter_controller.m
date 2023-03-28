function u_qp_filtered = get_qpfilter_controller(current_state, u_nom, params)    
    
    options = optimoptions('fmincon','Display','off','Algorithm','interior-point');
    % valuex queries the the value function from the precomputed brt
    valuex = eval_u(params.g,params.data(:,:,:,end),current_state);
    
    % dvdx computes the spatial derivative of the value function at the
    % current state
    dvdx = eval_u(params.g,params.derivatives,current_state);
    
    % optDst is a vector containing the optimal disturbance at the current
    % state as computed by the BRT
    optDst(1) = eval_u(params.g,params.worst_dist(1),current_state);
    optDst(2) = eval_u(params.g,params.worst_dist(2),current_state);
    
    %% TODO
    % Code in the quadratic program that implement the QP
    % min(u) 0.5*||u_nom -u||^2 st u is a safe control 
    % Why is dvdx needed? Isnt derivatives already dvdx?
    % What are optDst needed for?
    A = -dvdx(3)*params.dt;
    b = valuex + (params.dt * dvdx(1) .* (params.speed * cos(current_state(3)))) + ...
        (params.dt * dvdx(2) .* (params.speed * sin(current_state(3)))) + ... 
        (params.dt * dvdx(1) .* optDst(1)) + (params.dt * dvdx(2) .* optDst(2));
    u_qp_filtered = fmincon(@(u)0.5*(u-u_nom)^2, u_nom,[A],[b], [], [], [], [], [], options);
    
end