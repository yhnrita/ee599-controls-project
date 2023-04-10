function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the DubinsCar
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% TODO 
% Compute the optimal disturbance 

% % Minimize distance
% alpha = atan2(-deriv{2}, -deriv{1});
% dOpt{1} = (obj.dMax * cos(alpha)); % Compute the optimal disturbance in x
% dOpt{2} = (obj.dMax * sin(alpha)); % Compute the optimal disturbance in y

theta = atan2(deriv{2}, deriv{1});

    if strcmp(dMode, 'max')
      dOpt{1} = obj.dMax * cos(theta);
      dOpt{2} = obj.dMax * sin(theta);
    elseif strcmp(dMode, 'min')
      dOpt{1} = -obj.dMax * cos(theta);
      dOpt{2} = -obj.dMax * sin(theta);
    else
      error('Unknown dMode!')
    end
    
end