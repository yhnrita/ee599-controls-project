syms x_com_t(t) z_com_t(t) theta_com_t(t) dx_com_t(t) dz_com_t(t) dtheta_com_t(t)
syms x_com z_com theta_com dx_com dz_com dtheta_com
syms fx1 fz1 fx2 fz2
syms M_b I_b g f_pos_x1 f_pos_z1 f_pos_x2 f_pos_z2 l_b

M_b_val = 1;
I_b_val = 0.5;
g_val = 9.81;
f_pos_x1_val = 0.4;
f_pos_z1_val = 0;
f_pos_x2_val = -0.4;
f_pos_z2_val = 0;
l_b_val = 0.3;

param_vals = [M_b_val I_b_val g_val f_pos_x1_val f_pos_z1_val f_pos_x2_val f_pos_z2_val l_b_val];

%group symbolic variables
state_t = {x_com_t(t) z_com_t(t) theta_com_t(t) dx_com_t(t) dz_com_t(t) dtheta_com_t(t)};
state = {x_com z_com theta_com dx_com dz_com dtheta_com};

state_diff = {diff(x_com_t(t),t),diff(z_com_t(t),t),diff(theta_com_t(t),t)};
state_dot = {dx_com dz_com dtheta_com};

%dynamics dot_state = ?
f(1) = dx_com_t;
f(2) = dz_com_t;
f(3) = dtheta_com_t;

%dynamics for ddot_x_com, ddot_z_com, ddot_theta_com
f(4) = 1/M_b * (fx1+fx2); %ddot_x_com
f(5) = 1/M_b * (fz1+fz2) - g; %ddot_z_com
p_ta1 = f_pos_x1-x_com_t;
p_ta2 = f_pos_z1-z_com_t;
p_tb1 = f_pos_x2-x_com_t;
p_tb2 = f_pos_z2-z_com_t;
f(6) = -1 * 1/I_b * (p_ta1*fz1 - fx1*p_ta2 + p_tb1*fz2 - fx2*p_tb2);

%replace parameters and drop time dependence
f = subs(f, [M_b I_b g f_pos_x1 f_pos_z1 f_pos_x2 f_pos_z2 l_b], param_vals);
f = subs(f,state_t,state);
f = simplify(f);

%calculate linearization
A = jacobian(f,[state{:}]);
control_input = [fx1, fz1, fx2, fz2];
B = jacobian(f,control_input);

% Create rbt_state_fcn.m, robot dynamics
matlabFunction(transpose(f),'File','rbt_state_fcn',...
    'Vars',{transpose([state{:}]),transpose(control_input)})
% Create rbt_state_jacobian_fcn.m, robot dynamics jacobian
matlabFunction(A, B,'File','rbt_state_jacobian_fcn',...
    'Vars',{transpose([state{:}]),transpose(control_input)})