function u_nom = get_nominal_controller(xinit,params)
    % return the mpc based nominal controller
    X = myMpc(params.H, params.goalX, params.goalY, params.goalZ, params.dt, params.wMax, params.zMax, xinit, params.speed);
    u_nom = [X(5 * (params.H+1) + 1, 1); X(5 * (params.H+1) + params.H + 1, 1)];
end