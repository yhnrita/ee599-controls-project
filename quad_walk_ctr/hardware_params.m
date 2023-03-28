function [world, body, ctr, path] = hardware_params()
%% Casadi path
% Change to your casadi path
path.casadi = 'tool_box\casadi-windows-matlabR2016a-v3.5.5';

%% Simulation params
world.fk = 0.5; % friction
world.g = 9.81; % gravity constant


%% Controller params
ctr.phase_num = 4;
ctr.N = 80; % mpc period window
ctr.T = 1.3; % mpc period time
ctr.dt_val = (ctr.T/ctr.N) .* ones(ctr.N,1); % dt vector

ctr.max_jump_z = 0.55; % max jumping height, constraints
ctr.min_dump_z = 0.15; % min standing height
ctr.max_lift_vel_z = 6.5; % max jumping velocity
ctr.init_z = 0.3;

ctr.x_init_tar_val = [0; 0; 0; 0; 0; ctr.init_z]; % init state
ctr.dx_init_tar_val = [0; 0; 0; 0; 0; 0]; % init d state
ctr.x_final_tar_val = [0; -pi*0.3*0; pi*0.5; 1.5; 0; 0.2]; % final target state
ctr.dx_final_tar_val = [0; 0; 0; 0; 0; 0];

ctr.contact_state_ratio = ctr.N.*[0.35 0.15 0.475 0.025]; % pull, jump, flight, impact
ctr.contact_state_val = [ones(4, ctr.contact_state_ratio(1)),...
                         ones(4, ctr.contact_state_ratio(2)),...
                         0 * ones(4, ctr.contact_state_ratio(3)),...
                         0 * ones(4, ctr.contact_state_ratio(4))]; % no foot contact during last 2 phases
                 
% cost gains
ctr.weight.QX = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]';
ctr.weight.QN = [10 10 10, 50 50 50, 10 10 10, 10 10 10 ]';
ctr.weight.Qc = 1*[0.01 0.01 0.01]';
ctr.weight.Qf = [0.0001 0.0001 0.001]';

%% Robot params

body.state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
body.f_dim = 12; % number of dim of leg force, 3*4
body.fp_dim = 12; % number of dim of leg pos, 3*4


body.m = 5;
body.i_vec = [0.059150 0.101150 0.046240]*2;
body.i_mat = [body.i_vec(1) 0 0;... % roll
              0 body.i_vec(2) 0;... % pitch
              0 0 body.i_vec(3)]; % yaw
       
body.length = 0.34;
body.width = 0.26;

% foot motion range, in m
body.foot_x_range = 0.15;
body.foot_y_range = 0.15;
body.foot_z_range = 0.3;

% output force range
body.max_zforce = 1000;

% calaute hip position
body.hip_vec = [body.length/2; body.width/2; 0];
body.hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0];
body.hip_pos = body.hip_dir_mat .* repmat(body.hip_vec,1,4);
body.foot_pos = repmat([0; 0; -0.6*ctr.init_z],1,4); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
% ref foot pos at swing phase

body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1);

body.friction_cone = [1/world.fk, 0 -1;...
                      -1/world.fk, 0 -1;...
                      0, 1/world.fk, -1;...
                      0, -1/world.fk, -1];

body.foot_convex_hull = [1 0 0 -body.foot_x_range;
                        -1 0 0 -body.foot_x_range;
                        0 1 0 -body.foot_y_range;
                        0 -1 0 -body.foot_y_range;
                        0 0 1 -min_dump_z;
                        0 0 -1 -body.foot_z_range];
                

end