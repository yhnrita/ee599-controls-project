% function X = myMpc(H, goalX, goalY, dt, wMax, xinit, v)
% 
%     %% Optimization variables
%     % [x(1), x(2), ..., x(101),y(1), y(2), ...y(101), theta(1), theta(2), ...,theta(101),
%     % thetadot(1), thetadot(2), ..., thetadot(100)]
%     % Total number of varaibles: 101*3 + 100 = 403 
% 
%     % setup number of decision variables
%     num_state_decision = 3*(H+1);
%     num_control_decision = 1*(H);
%     num_decision = num_state_decision + num_control_decision;
%     
% 
%     %% setup inital condition
%     x0 = zeros(num_decision,1);
%     x0(1) = xinit(1);
%     x0(H+2) = xinit(2);
%     x0(3*(H+1)+1) = xinit(3);
%     
%     %% Setup control bounds
%     % Lower bounds
%     LB = [-100*ones(H+1, 1); -100*ones(H+1, 1);-100*ones(H+1, 1); -wMax*ones(H, 1)];
%     UB = -1*LB;
%     
%     %% Perform the optimization and solve the MPC problem
%     options = optimoptions('fmincon','Display','off', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 300000);
%     nonlcon = @(z)dynConst(z,v,xinit,dt);
%     X = fmincon(@(x) myfun(x, H, goalX, goalY), x0,[],[], [], [], LB(:, 1), UB(:, 1), nonlcon, options);
% 
% end
% 
% %% Setup cost function
% function f = myfun(x, H, goalX, goalY)
%     % Penalize the distance from the goal state
%     f = sum((x(1:H+1) - goalX).^2) + sum((x(H+2:2*H+2) - goalY).^2);
% end


function X = myMpc(H, goalX, goalY, goalZ, dt, wMax, zMax, xinit, v)

    %% Optimization variables
    % [x(1), x(2), ..., x(101),y(1), y(2), ...y(101), theta(1), theta(2), ...,theta(101),
    % thetadot(1), thetadot(2), ..., thetadot(100)]
    % Total number of varaibles: 101*3 + 100 = 403 

    %% setup number of decision variables
    num_state_decision = 5*(H+1);
    num_control_decision = 2*(H);
    num_decision = num_state_decision + num_control_decision;
    
    %% setup initial condition
    x0 = zeros(num_decision,1);
    x0(                   1 )   = xinit(1); % x
    x0(     (H+1)       + 1 )   = xinit(2); % y
    x0( 2 * (H+1)       + 1 )   = xinit(3); % z
    x0( 5 * (H+1)       + 1 )   = xinit(4); % theta
    x0( 5 * (H+1)  + H  + 1 )   = xinit(5); % z speed
    
    %% Setup control bounds
    % Lower bounds
    LB = [-100*ones(H+1, 1); -100*ones(H+1, 1); -100*ones(H+1, 1); -100*ones(H+1, 1); -100*ones(H+1, 1); -wMax*ones(H, 1); -zMax*ones(H, 1)];
    UB = -1*LB;
    
    %% Perform the optimization and solve the MPC problem
    options = optimoptions('fmincon','Display','off', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 300000);
    nonlcon = @(z)dynConst(z,v,xinit,dt);
    X = fmincon(@(x) myfun(x, H, goalX, goalY, goalZ), x0, [], [], [], [], LB(:, 1), UB(:, 1), nonlcon, options);

end

%% Setup cost function
function f = myfun(x, H, goalX, goalY, goalZ)
    % Penalize the distance from the goal state
    f =   sum( ( x(           1 :    H+1  ) - goalX).^2 ) ...
        + sum( ( x(    H+1  + 1 : 2*(H+1) ) - goalY).^2 ) ...
        + sum( ( x( 2*(H+1) + 1 : 3*(H+1) ) - goalZ).^2 );
end