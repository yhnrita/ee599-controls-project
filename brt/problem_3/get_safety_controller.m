function u_filtered = get_safety_controller(x_curr,u_nom,params)  
   
    %% TODO
    % Code in the least restrictive safety filter that returns the safe
    % controller if the nominal controller leads the next state into an
    % unsafe region
    signed_distance = eval_u(params.g,params.data(:,:,:,end),x_curr);
    if signed_distance <= 0.001  % the system will end in unsafe state!
            u_filtered = eval_u(params.g,params.safety_controller,x_curr); % get safety controller
    else
            u_filtered = u_nom;
    end

end