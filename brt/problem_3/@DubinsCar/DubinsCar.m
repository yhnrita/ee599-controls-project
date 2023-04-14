classdef DubinsCar < DynSys
  properties
    % Angle bounds
    wRange
    
    speed % Constant speed
    
    % Disturbance
    dMax
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = DubinsCar(x, wRange, speed, dMax, dims)
      % obj = DubinsCar(x, wMax, speed, dMax, dims)
      %     Dubins Car class
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(x_3) + d1
      %    \dot{x}_2 = v * sin(x_3) + d2
      %    \dot{x}_3 = u
      %         u \in [-wMax, wMax]
      %         d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   thetaMin   - minimum angle
      %   thetaMax   - maximum angle
      %   v - speed
      %
      % Output:
      %   obj       - a DubinsCar2D object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        wRange = [-1 1];
      end
      
      if nargin < 3
        speed = 1;
      end
     
      
      if nargin < 5
        dims = 1:3;
      end
      
      if numel(wRange) <2
          wRange = [-wRange; wRange];
      end
      
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)];     % Position dimensions
      obj.nx = length(dims);                            % Number of state dimensions
      obj.nu = 1;                                       % Number of control inputs
      obj.nd = 2;                                       % Number of disturbance inputs
    
      % Set the initial state and its history
      obj.x = x;
      obj.xhist = obj.x;
    
      % Set other properties for the DubinsCar object
      obj.wRange = wRange;                              % Steering angle range
      obj.speed = speed;                                % Constant speed
      obj.dMax = dMax;                                  % Maximum disturbance
      obj.dims = dims;                                  % Active dimensions
    end
    
  end % end methods
end % end classdef


% classdef DubinsCar < DynSys
%   properties
%     % Control input bounds
%     FxRange % Range of force in x-axis
%     FyRange % Range of force in y-axis
%     TRange  % Range of torque
%     
%     speed   % Constant speed
% 
%     % Car properties
%     mass    % Mass of the car
%     inertia % Moment of inertia about the z-axis
% 
%     % Disturbance
%     dMax    % Disturbance range
%     
%     % Dimensions that are active
%     dims    % Dimensions that are active
%   end
%   
%   methods
%     function obj = DubinsCar(x, FxRange, FyRange, TRange, mass, inertia, dMax, dims)
%       % obj = DubinsCar(x, FxRange, FyRange, TRange, mass, inertia, dMax, dims)
%       %     Dubins Car class
%       %
%       % Dynamics:
%       %    \dot{x}_1 = Fx/m
%       %    \dot{x}_2 = Fy/m
%       %    \dot{x}_3 = T/I
%       %         Fx \in [FxMin, FxMax]
%       %         Fy \in [FyMin, FyMax]
%       %         T  \in [TMin,  TMax]
%       %         d \in [-dMax, dMax]
%       %
%       % Inputs:
%       %   x      - state: [xpos; ypos; theta]
%       %   FxRange - range of force in x-axis
%       %   FyRange - range of force in y-axis
%       %   TRange  - range of torque
%       %   mass    - mass of the car
%       %   inertia - moment of inertia about the z-axis
%       %   dMax    - disturbance range
%       %   dims    - dimensions that are active
%       %
%       % Output:
%       %   obj       - a DubinsCar object
% 
%       % Check if the initial state has the right dimension
%       if numel(x) ~= obj.nx
%         error('Initial state does not have right dimension!');
%       end
%       
%       % Ensure the initial state is a column vector
%       if ~iscolumn(x)
%         x = x';
%       end
% 
%       % Assign default values if arguments are not provided
%       if nargin < 2
%         FxRange = [-1 1];
%       end
%       
%       if nargin < 3
%         FyRange = [-1 1];
%       end
%       
%       if nargin < 4
%         TRange = [-1 1];
%       end
% 
%       if nargin < 5
%         mass = 1;
%       end
% 
%       if nargin < 6
%         inertia = 1;
%       end
% 
%       if nargin < 7
%         dMax = 0.1;
%       end
%       
%       if nargin < 8
%         dims = 1:3;
%       end
%       
%       % Basic vehicle properties
%       obj.pdim = [find(dims == 1) find(dims == 2)];   % Position dimensions
%       obj.nx = length(dims);                          % Number of state dimensions
%       obj.nu = 3;                                     % Number of control inputs
%       obj.nd = 2;                                     % Number of disturbance inputs
%       
%       % Set the initial state and its history
%       obj.x = x;
%       obj.xhist = obj.x;
%       
%       obj.FxRange = FxRange;
%       obj.FyRange = FyRange;
%       obj.TRange = TRange;
%       obj.mass = mass;
%       obj.inertia = inertia;
% 
%       obj.speed = 1;                                  % Constant speed
% 
%       obj.dMax = dMax;                                % Maximum disturbance
%       obj.dims = dims;                                % Active dimensions
%     end
% end       % end methods
% end       % end classdef
