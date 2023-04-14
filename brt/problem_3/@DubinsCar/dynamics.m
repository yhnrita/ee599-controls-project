function dx = dynamics(obj, ~, x, u, d)
    % Dynamics of the Dubins Car
    %    \dot{x}_1 = v * cos(x_3) + d1
    %    \dot{x}_2 = v * sin(x_3) + d2
    %    \dot{x}_3 = w
    %   Control: u = w;
    %
    % Mo Chen, 2016-06-08
    
    if nargin < 5
      d = [0; 0];
    end
    
    if iscell(x)

      dx = cell(length(obj.dims), 1);
      
      for i = 1:length(obj.dims)
        dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));      %%%%%%%%%%%%%%%%%%%%%%%%%%
      end
    else
      printf("here");
      dx = zeros(obj.nx, 1);
      
      dx(1) = obj.speed * cos(x(3)) + d(1);
      dx(2) = obj.speed * sin(x(3)) + d(2);
      dx(3) = u;
    end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)
    switch dim
      case 1
        dx = obj.speed * cos(x{dims==3}) + d{1};
      case 2
        dx = obj.speed * sin(x{dims==3}) + d{2};
      case 3
        dx = u;
      otherwise
        error('Only dimension 1-3 are defined for dynamics of DubinsCar!')
    end
end




% function dx = dynamics(obj, ~, x, u, d)
%     % Dynamics of the Dubins Car
%     %    \dot{x}_1 = Fx/m
%     %    \dot{x}_2 = Fy/m
%     %    \dot{x}_3 = T/I
%     %   Control: u = [Fx; Fy; T];
%     
%     % Check if disturbance argument is provided, if not, set to zero
%     if nargin < 5
%       d = [0; 0];
%     end
% 
%     % Check if the state is in cell format
%     if iscell(x)
%       dx = cell(length(obj.dims), 1);
%       
%       % Loop through each dimension and call the cell helper function
%       for i = 1:length(obj.dims)
%         dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
%       end
%       obj.speed = obj.speed + u{1}/obj.mass;            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Only Fx here is used to demo
%     else
%       % Initialize the derivative vector
% %       dx = zeros(obj.nx, 1);
% %       
% %       % Calculate the derivative of x_1 using Fx, mass, and disturbance
% %       dx(1) = u(1)/obj.mass + d(1); % \dot{x}_1 = Fx/m + d1
% %       % Calculate the derivative of x_2 using Fy, mass, and disturbance
% %       dx(2) = u(2)/obj.mass + d(2); % \dot{x}_2 = Fy/m + d2
% %       % Calculate the derivative of x_3 using torque and inertia
% %       dx(3) = u(3)/obj.inertia;     % \dot{x}_3 = T/I
%     end
% end
% 
% % Helper function to calculate the derivative for each dimension
% function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)
%     switch dim                          % Switch case for each dimension
%       case 1
%         %dx = u{1}/obj.mass + d{1};      % Calculate the derivative of x_1 using Fx, mass, and disturbance
%         dx = obj.speed .* cos(x{3}) + d{1};
%       case 2
%         %dx = u{2}/obj.mass + d{2};      % Calculate the derivative of x_2 using Fy, mass, and disturbance
%         dx = obj.speed .* sin(x{3}) + d{2};
%       case 3
%         dx = u{3}/obj.inertia;          % Calculate the derivative of x_3 using torque and inertia
%       otherwise
%         % Throw an error if an invalid dimension is passed
%         error('Only dimension 1-3 are defined for dynamics of DubinsCar!')
%     end
% end

