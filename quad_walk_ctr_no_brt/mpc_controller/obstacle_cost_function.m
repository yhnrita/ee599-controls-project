%% Setup cost function
function c = obstacle_cost_function(x, H, goalX, goalY, obsCenterX, obsCenterY, obsSizeR)
    % Penalize the distance from the goal state
    f1 = sum((x(1:H+1) - goalX).^2) + sum((x(2*H+2:3*H+2) - goalY).^2);

    % Penalize if the robot goes inside the obstacle;
    f2 = sum(max(obsSizeR - sqrt(((x(1:H+1) - obsCenterX).^2) + ((x(2*H+2:3*H+2) - obsCenterY).^2)), 0));

    c = f1 + 5*f2;
end