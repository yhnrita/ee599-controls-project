function u_qp_filtered = get_qpfilter_controller(current_state, u_nom, params)    
    
    options = optimoptions('fmincon','Display','off','Algorithm','interior-point');
    % valuex queries the the value function from the precomputed brt
    valuex = eval_u(params.g,params.data(:,:,:,:,:,end),current_state);
    
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

    %% Planer Car Interesting
%     A = -dvdx(3)*params.dt;
%     b = valuex + (params.dt * dvdx(1) .* (params.speed * cos(current_state(3)))) + ...
%         (params.dt * dvdx(2) .* (params.speed * sin(current_state(3)))) + ... 
%         (params.dt * dvdx(1) .* optDst(1)) + (params.dt * dvdx(2) .* optDst(2));


    %% Planer Car Mine
%     A = - dvdx(3);
%     
%     b =  valuex + (dvdx(1) * (params.speed * cos(current_state(3)) + optDst(1)) ...
%         + dvdx(2) * (params.speed * sin(current_state(3)) + optDst(2)) );



    %% 3D Car seems right
    A = [- dvdx(4), - dvdx(5)];
    
    b =  valuex + (dvdx(1) * (params.speed * cos(current_state(4)) + optDst(1)) ...
        + dvdx(2) * (params.speed * sin(current_state(4)) + optDst(2)) ...
        + dvdx(3) * ( current_state(5)) );








    %% 3D Car interesting version
%     A = [-dvdx(4)*params.dt, -dvdx(5)*params.dt];
% 
%     b = valuex + (params.dt * dvdx(1) .* (params.speed * cos(current_state(4)))) + ...
%         (params.dt * dvdx(2) .* (params.speed * sin(current_state(4)))) + ... 
%         (params.dt * dvdx(1) .* optDst(1)) + (params.dt * dvdx(2) .* optDst(2)) + ...
%         params.dt * dvdx(3) .* current_state(5);
    



    u_qp_filtered = fmincon(@(u)( 0.5*(u(1)-u_nom(1))^2 + 0.5*(u(2)-u_nom(2))^2 ), u_nom,[A],[b], [], [], [], [], [], options);




    %% Two fmincon version
%     A1 = [- dvdx(4)];
%     
%     b1 =  valuex + (dvdx(1) * (params.speed * cos(current_state(4)) + optDst(1)) ...
%         + dvdx(2) * (params.speed * sin(current_state(4)) + optDst(2)) );
% 
%     A2 = [- dvdx(5)];
%     
%     b2 =  valuex + ( current_state(5) );

%     u_qp_filtered(1) = fmincon(@(u)( 0.5*(u-u_nom(1))^2 ), u_nom(1),[A1],[b1], [], [], [], [], [], options);    
%     u_qp_filtered(2) = fmincon(@(u)( 0.5*(u-u_nom(2))^2 ), u_nom(2),[A2],[b2], [], [], [], [], [], options);
%     0.5*(u_qp_filtered(2)-u_nom(2))^2
%     u_qp_filtered(2)
%     u_qp_filtered = [u_qp_filtered(1);u_qp_filtered(2)];
%     
%     ( 0.5*(u_qp_filtered(1)-u_nom(1))^2 + 0.5*(u_qp_filtered(2)-u_nom(2))^2 )




%     valuex
    if ( valuex < 1 )
        u_qp_filtered(2) = 5;
    end
    if ( u_qp_filtered(2) < -1 )
        u_qp_filtered(2) = -5;
    end

    
end