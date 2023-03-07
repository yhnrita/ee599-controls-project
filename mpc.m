function X = mpc(H, goalX, goalY, obsCenterX, obsSizeX, obsCenterY, obsSizeY, dt, vMax, xinit)

    %% Optimization variables
    % [x(1), x(2), ..., x(101), vX(1), vX(2), ..., vX(100), y(1), y(2), ...,
    % y(101), vY(1), vY(2), ..., vY(100)]
    % Total number of varaibles: (101 + 100)*2 = 402 
    
    %% Setup dynamics constraints
    % x(t+1) - [x(t) + dt * vX(t)] = 0
    % y(t+1) - [y(t) + dt * vY(t)] = 0
    
    % x state constraints
    AeqX = [-1*eye(H, H+1), -dt*eye(H)]; 
    for i=1:H
        AeqX(i, i+1) = 1;
    end
    BeqX = zeros(H, 1);
    
    % y state constraints
    AeqY = [-1*eye(H, H+1), -dt*eye(H)]; 
    for i=1:H
        AeqY(i, i+1) = 1;
    end
    BeqY = zeros(H, 1);
    
    % Merge the two constraints
    Aeq = blkdiag(AeqX, AeqY);
    Beq = [BeqX; BeqY];
    
    %% Setup initial state constraints
    init_state = zeros(2, size(Aeq, 2));
    init_state(1, 1) = 1;
    init_state(2, 2*H+2) = 1;
    Aeq = [Aeq ; init_state];
    Beq = [Beq ; xinit];
    
    %% Setup control bounds
    % Lower bounds
    LB = [-100*ones(H+1, 1); -vMax*ones(H, 1); -100*ones(H+1, 1); -vMax*ones(H, 1)];
    UB = -1*LB;
    
    %% Perform the optimization and solve the MPC problem
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 300000);
    X = fmincon(@(x) myfun(x, H, goalX, goalY, obsCenterX, obsSizeX, obsCenterY, obsSizeY), ones(size(LB)),[],[], Aeq, Beq, LB(:, 1), UB(:, 1), [], options);

end

%% Setup cost function
function f = myfun(x, H, goalX, goalY, obsCenterX, obsSizeX, obsCenterY, obsSizeY)
    % Penalize the distance from the goal state
    f1 = sum((x(1:H+1) - goalX).^2) + sum((x(2*H+2:3*H+2) - goalY).^2);

    % Penalize if the robot goes inside the obstacle
%     obs_signdist = max(abs(x(1:H+1) - obsCenterX) - obsSizeX, abs(x(2*H+2:3*H+2) - obsCenterY) - obsSizeY);
%     f2 = sum(abs(min(obs_signdist, 0)));
    f2 = sum(max(obsSizeX - sqrt(((x(1:H+1) - obsCenterX).^2) + ((x(2*H+2:3*H+2) - obsCenterY).^2)), 0));

    f = f1 + 5*f2;
end