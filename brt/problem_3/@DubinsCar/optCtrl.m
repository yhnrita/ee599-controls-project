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
%uOpt = (deriv{obj.dims==3}>=0)*(obj.wRange(2)) + (deriv{obj.dims==3}<0)*(obj.wRange(1));
if strcmp(uMode, 'min')
  uOpt{1} = (deriv{obj.dims==1}>=0)*obj.FxRange(1) + (deriv{obj.dims==1}<0)*obj.FxRange(2);
  uOpt{2} = (deriv{obj.dims==2}>=0)*obj.FyRange(1) + (deriv{obj.dims==2}<0)*obj.FyRange(2);
  uOpt{3} = (deriv{obj.dims==3}>=0)*obj.TRange(1) + (deriv{obj.dims==3}<0)*obj.TRange(2);
else % uMode is 'max'
  uOpt{1} = (deriv{obj.dims==1}>=0)*obj.FxRange(2) + (deriv{obj.dims==1}<0)*obj.FxRange(1);
  uOpt{2} = (deriv{obj.dims==2}>=0)*obj.FyRange(2) + (deriv{obj.dims==2}<0)*obj.FyRange(1);
  uOpt{3} = (deriv{obj.dims==3}>=0)*obj.TRange(2) + (deriv{obj.dims==3}<0)*obj.TRange(1);
end

end