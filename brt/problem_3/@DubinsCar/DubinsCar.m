% classdef DubinsCar < DynSys
%   properties
%     % Angle bounds
%     wRange
%     
%     speed % Constant speed
%     
%     % Disturbance
%     dMax
%     
%     % Dimensions that are active
%     dims
%   end
%   
%   methods
%     function obj = DubinsCar(x, wRange, speed, dMax, dims)
%       % obj = DubinsCar(x, wMax, speed, dMax, dims)
%       %     Dubins Car class
%       %
%       % Dynamics:
%       %    \dot{x}_1 = v * cos(x_3) + d1
%       %    \dot{x}_2 = v * sin(x_3) + d2
%       %    \dot{x}_3 = u
%       %         u \in [-wMax, wMax]
%       %         d \in [-dMax, dMax]
%       %
%       % Inputs:
%       %   x      - state: [xpos; ypos]
%       %   thetaMin   - minimum angle
%       %   thetaMax   - maximum angle
%       %   v - speed
%       %
%       % Output:
%       %   obj       - a DubinsCar2D object
%       
%       if numel(x) ~= obj.nx
%         error('Initial state does not have right dimension!');
%       end
%       
%       if ~iscolumn(x)
%         x = x';
%       end
%       
%       if nargin < 2
%         wRange = [-1 1];
%       end
%       
%       if nargin < 3
%         speed = 1;
%       end
%      
%       
%       if nargin < 5
%         dims = 1:3;
%       end
%       
%       if numel(wRange) <2
%           wRange = [-wRange; wRange];
%       end
%       
%       
%       % Basic vehicle properties
%       obj.pdim = [find(dims == 1) find(dims == 2)];     % Position dimensions
%       obj.nx = length(dims);                            % Number of state dimensions
%       obj.nu = 1;                                       % Number of control inputs
%       obj.nd = 2;                                       % Number of disturbance inputs
%     
%       % Set the initial state and its history
%       obj.x = x;
%       obj.xhist = obj.x;
%     
%       % Set other properties for the DubinsCar object
%       obj.wRange = wRange;                              % Steering angle range
%       obj.speed = speed;                                % Constant speed
%       obj.dMax = dMax;                                  % Maximum disturbance
%       obj.dims = dims;                                  % Active dimensions
%     end
%     
%   end % end methods
% end % end classdef


classdef DubinsCar < DynSys
  properties
    % Angle bounds
    wRange
    
    speed % Constant speed

    zRange % Constant speed
    
    % Disturbance
    dMax
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = DubinsCar(x, wRange, zRange, speed, dMax, dims)
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
     
      
      if nargin < 6
        dims = 1:5;
      end
      
      if numel(wRange) <2
          wRange = [-wRange; wRange];
      end

      if numel(zRange) <2
          zRange = [-zRange; zRange];
      end
      
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2) find(dims == 3)];     % Position dimensions
      obj.nx = length(dims);                                            % Number of state dimensions
      obj.nu = 1;                                                       % Number of control inputs
      obj.nd = 2;                                                       % Number of disturbance inputs
    
      % Set the initial state and its history
      obj.x = x;
      obj.xhist = obj.x;
    
      % Set other properties for the DubinsCar object
      obj.wRange = wRange;                              % Steering angle range
      obj.zRange = zRange;                              % zRange
      obj.speed = speed;                                % Constant speed
      obj.dMax = dMax;                                  % Maximum disturbance
      obj.dims = dims;                                  % Active dimensions
    end
    
  end % end methods
end % end classdef
