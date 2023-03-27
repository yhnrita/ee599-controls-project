%% SRB dynamic model of quadruped in 3d space
% Shuang Peng 02/2023 

% Clear command window, workspace, and close all figure windows
clc;
clear;
close all;

% Turn off all warnings
warning off;

% Add path to the CasADi library
addpath('C:\My_Data\Course\EE599_Bansal\Project\casadi-windows-matlabR2016a-v3.5.5');

% Import all CasADi functions and classes
import casadi.*;

% Define world parameters
world.fk = 0.5; % Friction coefficient
world.g = 9.81; % Gravitational constant

% Define body properties
body.m = 5*2; % Total body mass
init_z = 0.3; % Initial z-coordinate

% Define body's inertia tensor components
i_vec = [0.059150 0.101150 0.046240]*2;
body.i_mat = [i_vec(1) 0 0;... % roll
0 i_vec(2) 0;... % pitch
0 0 i_vec(3)]; % yaw

% Define body dimensions
body.length = 0.34;
body.width = 0.26;

% Define foot motion range in meters
body.foot_x_range = 0.15;
body.foot_y_range = 0.15;
body.foot_z_range = 0.3;

% Define maximum z-force
body.max_zforce = 1000;

% Calculate hip positions for the quadruped robot
hip_vec = [body.length/2; body.width/2; 0]; % Vector defining the position of hips
hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0]; % Direction matrix for hip positions
body.hip_pos = hip_dir_mat .* repmat(hip_vec,1,4); % Calculate hip positions for all four legs

% Initialize foot positions for the quadruped robot
body.foot_pos = repmat([0; 0; -0.6*init_z],1,4); % Define initial foot positions for all four legs

% Calculate reference foot positions at swing phase
body.phip_swing_ref = body.hip_pos + body.foot_pos; % Calculate reference foot positions
body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1); % Reshape the reference foot positions into a single column vector

% The body.phip_swing_ref Matrix
% [0.1700    0.1700   -0.1700   -0.1700;
%  0.1300   -0.1300    0.1300   -0.1300;
% -0.1800   -0.1800   -0.1800   -0.1800];

% Define dimensions for state, leg force, and leg position
state_dim = 12; % Number of dimensions for the state
f_dim = 12; % Number of dimensions for leg force
fp_dim = 12; % Number of dimensions for leg position

% Define CasADi symbolic variables for optimization
x_k = SX.sym('x_k', state_dim, 1); % State variable
f_k = SX.sym('f_k', f_dim, 1); % Foot force variable
fp_k = SX.sym('fp_k', fp_dim, 1); % Foot position variable

% Calculate rotation matrix for roll, pitch, and yaw
rot_mat_zyx = rot_zyx(x_k(1:3)); % Rotation matrix for ZYX (yaw, pitch, roll) Euler angles

% Define trigonometric functions for yaw and pitch
s_yaw = sin(x_k(3)); % Sine of yaw
c_yaw = cos(x_k(3)); % Cosine of yaw
t_yaw = tan(x_k(3)); % Tangent of yaw
s_pitch = sin(x_k(2)); % Sine of pitch
c_pitch = cos(x_k(2)); % Cosine of pitch
t_pitch = tan(x_k(2)); % Tangent of pitch

% Calculate inverse rotation matrices for linear and nonlinear components
inv_rot_linear = [c_yaw s_yaw 0; % Inverse rotation matrix for linear components
                  -1*s_yaw c_yaw 0;
                  0 0 1];
inv_rot_nonlinear = [c_yaw/c_pitch s_yaw/c_pitch 0; % Inverse rotation matrix for nonlinear components
                     -1*s_yaw c_yaw 0;
                     c_yaw*t_pitch s_yaw*t_pitch 1];
                
% Convert the inertia tensor from the local coordinate system to the world coordinate system
i_mat_w = rot_mat_zyx*body.i_mat*rot_mat_zyx'; % Inertia matrix in world coordinates
i_mat_w_inv = eye(3)/i_mat_w; % Inverse of the inertia matrix in world coordinates


% Define A, B, G matrices for the state-space representation of the system
A = [zeros(3) zeros(3) inv_rot_linear zeros(3);...
     zeros(3) zeros(3) zeros(3) eye(3);...
     zeros(3) zeros(3) zeros(3) zeros(3);...
     zeros(3) zeros(3) zeros(3) zeros(3)];
% A = [z3 z3 inv_rot z3;
%      z3 z3 z3      I3
%      z3 z3 z3      z3
%      z3 z3 z3      z3];

B = [zeros(3) zeros(3) zeros(3) zeros(3);...
     zeros(3) zeros(3) zeros(3) zeros(3);...
     i_mat_w_inv*skew_mat(fp_k(1:3)), i_mat_w_inv*skew_mat(fp_k(4:6)), i_mat_w_inv*skew_mat(fp_k(7:9)), i_mat_w_inv*skew_mat(fp_k(10:12));...
     eye(3)/body.m, eye(3)/body.m, eye(3)/body.m, eye(3)/body.m];
% B = [z3 z3 z3 z3
%      z3 z3 z3 z3
%      (I_m)^-1*[f_pos]x
%      I3/m I3/m I3/m I3/m];

G = zeros(12,1); % Vector G representing gravitational effects
G(12) = -1*world.g; % Assign the gravitational acceleration to the last element

d_x = A*x_k + B*f_k + G; % Calculate the change in state (state derivative) using A, B, and G matrices

% Create a dynamic function mapping state, leg_force, and foot_pos to the state derivative
dyn_f = Function('dyn_f',{x_k;f_k;fp_k},{d_x},{'state','leg_force','foot_pos'},{'d_state'});

% Display the initial foot positions
body.foot_pos

% Perform a simple test of the dynamic function
x_init = [0;0.0;0; 0.0;0.0;0.5 ;0;0;0; 0;0;0]; % Define an initial state
% Output is the state derivative
dyn_f(x_init, zeros(12,1), zeros(12,1))
