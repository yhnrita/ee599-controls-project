function stop = stopping_criteria(current_state, params)
%     %% 2D
%     stop = (norm(current_state(1:2) - [params.goalX; params.goalY]) - params.goalR) < 0;

    %% 3D
    stop = (norm([current_state(1:2); current_state(4)] - [params.goalX; params.goalY; params.goalZ]) - params.goalR) < 0;
end