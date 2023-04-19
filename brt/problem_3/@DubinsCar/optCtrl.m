function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% TODO 
% Compute the optimal controller 

%% Method 1
% % Maximize distance
% uOpt = abs(deriv{3} * obj.wRange(1));
% uOpt = (deriv{obj.dims==3}>=0)*(obj.wRange(2)) + (deriv{obj.dims==3}<0)*(obj.wRange(1));

%% Method 2
% The optimal control for DubinsCar is to minimize or maximize the rate of
% change in the heading angle based on the sign of the derivative in x_3
% if strcmp(uMode, 'min')
%   uOpt = (deriv{3} >= 0) * obj.wRange(1) + (deriv{3} < 0) * obj.wRange(2);
% elseif strcmp(uMode, 'max')
%   uOpt = (deriv{3} >= 0) * obj.wRange(2) + (deriv{3} < 0) * obj.wRange(1);
% else
%   error('Unknown uMode!')
% end

%% New Dynamics
% if strcmp(uMode, 'min')
%   uOpt{1} = (deriv{1}>=0) * obj.FxRange(1) + (deriv{1}<0) * obj.FxRange(2);
%   uOpt{2} = (deriv{2}>=0) * obj.FyRange(1) + (deriv{2}<0) * obj.FyRange(2);
%   uOpt{3} = (deriv{3}>=0) * obj.TRange(1)  + (deriv{3}<0) * obj.TRange(2);
% else % uMode is 'max'
%   uOpt{1} = (deriv{1}>=0) * obj.FxRange(2) + (deriv{1}<0) * obj.FxRange(1);
%   uOpt{2} = (deriv{2}>=0) * obj.FyRange(2) + (deriv{2}<0) * obj.FyRange(1);
%   uOpt{3} = (deriv{3}>=0) * obj.TRange(2)  + (deriv{3}<0) * obj.TRange(1);
% end

%% New Dynamics 2
uOpt{1} = (deriv{4} >= 0) * obj.wRange(1) + (deriv{4} < 0) * obj.wRange(2);
uOpt{2} = (deriv{5} >= 0) * obj.zRange(1) + (deriv{5} < 0) * obj.zRange(2);

end
