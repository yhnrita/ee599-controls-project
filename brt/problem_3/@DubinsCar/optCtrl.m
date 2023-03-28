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
% Maximize distance
% uOpt = abs(deriv{3} * obj.wRange(1));
uOpt = (deriv{obj.dims==3}>=0)*(obj.wRange(2)) + (deriv{obj.dims==3}<0)*(obj.wRange(1));

end