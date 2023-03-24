%% Control a 2d quadruped pose with nmpc, modified from MPC toolbox's FlyingRobot example
% MPC toolbox required - https://www.mathworks.com/products/model-predictive-control.html
% Shuang Peng 01/2023 

clear all; close all; clc;
addpath('quad_2d_dyn\');

% Nonlinear MPC is an ideal tool for trajectory planning problems because
% it solves an open-loop constrained nonlinear optimization problem given
% the current plant states. With the availability of a nonlinear dynamic
% model, MPC can make more accurate decisions.
%
% Create a nonlinear MPC object with |6| states, |6| outputs, and |4|
% inputs. By default, all the inputs are manipulated variables (MVs).
clc;
clear;

nx = 6;
ny = 6;
nu = 4;
nlobj = nlmpc(nx,ny,nu);

%%
% Specify the prediction model state function using the function name. You
% can also specify functions using a function handle. For details on the
% state function, open |FlyingRobotStateFcn.m|. For more information on
% specifying the prediction model, see
% <docid:mpc_ug#mw_cda3c7d0-97e5-4c33-9ec5-dcf440b4c5cf>.
nlobj.Model.StateFcn = "rbt_state_fcn";

%%
% Specify the Jacobian of the state function using a function handle. It is
% best practice to provide an analytical Jacobian for the prediction model.
% Doing so significantly improves simulation efficiency. For details on the
% Jacobian function, open |FlyingRobotStateJacobianFcn.m|.
nlobj.Jacobian.StateFcn = @rbt_state_jacobian_fcn;

%%
% For this example, the target prediction time is |12| seconds. Therefore,
% specify a sample time of |0.4| seconds and prediction horizon of |30|
% steps.
Ts = 0.4;
p = 30;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;

%%
% To perform trajectory planning instead of feedback control, use the
% maximum control horizon, which provides the maximum number of decision
% variables for the optimization problem. Since trajectory planning usually
% runs at a much slower sampling rate than a feedback controller, the extra
% computation load introduced by a large control horizon can be tolerated.
% Set the control horizon equal to the prediction horizon.
nlobj.ControlHorizon = p;

%%
% A trajectory planning problem usually involves a nonlinear cost function,
% which can be used to find the shortest distance, the maximal profit, or
% as in this case, the minimal fuel consumption. Because the thrust value
% is a direct indicator of fuel consumption, compute the fuel cost as the
% sum of the thrust values used across the prediction horizon. Specify this
% cost function using an anonymous function handle. For more information on
% specifying cost functions, see
% <docid:mpc_ug#mw_c432ddbe-6685-4cf2-be0e-2d46e07fed51>.
nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));

%%
% For this example, the custom cost function replaces the default cost
% function that is typically used in feedback control.
nlobj.Optimization.ReplaceStandardCost = true;

%%
% The goal of the maneuver is to park the robot at |[0,0]| with an angle of
% |0| radians at the 12th second. Specify this goal as equality constraints
% on the states, where every position and velocity state at the last
% prediction step should be zero. For more information on specifying
% constraint functions, see
% https://www.mathworks.com/help/mpc/ug/specify-constraints-for-nonlinear-mpc.html
X_final_state = [0; 0.5; 0; 0; 0; 0];
nlobj.Optimization.CustomEqConFcn = @(X,U,data) [X(end,1) - X_final_state(1);...
                                                 X(end,2) - X_final_state(2);...
                                                 X(end,3) - X_final_state(3);...
                                                 X(end,4) - X_final_state(4);...
                                                 X(end,5) - X_final_state(5);...
                                                 X(end,6) - X_final_state(6)];

%%
% It is best practice to provide analytical Jacobian functions for your
% custom cost and constraint functions as well. However, this example
% intentionally skips them so that their Jacobian is computed by the
% nonlinear MPC controller using the built-in numerical perturbation
% method.

%%
% Each thrust has an operating range between |0| and |1|, which is
% translated into lower and upper bounds on the MVs.
nlobj.MV = struct('Min',{-30;0;-30;0},'Max',{30;30;30;30});


%%
% Specify the initial conditions for the robot.
x0 = [0;0;0;0;0;0];  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero

%%
% It is best practice to validate the user-provided model, cost, and
% constraint functions and their Jacobians. To do so, use the
% |validateFcns| command.
validateFcns(nlobj,x0,u0);

%%
% The optimal state and MV trajectories can be found by calling the
% |nlmpcmove| command once, given the current state |x0| and last MV |u0|.
% The optimal cost and trajectories are returned as part of the |info|
% output argument.
[~,~,info] = nlmpcmove(nlobj,x0,u0);

%%
% Plot the optimal trajectory. The optimal cost is 7.8.
%FlyingRobotPlotPlanning(info);
state_arr = info.Xopt;
input_arr = info.MVopt;
t_arr = info.Topt;

%plot_rbt_state_input;

%% Feedback Control for Path Following
% After the optimal trajectory is found, a feedback controller is required
% to move the robot along the path. In theory, you can apply the optimal MV
% profile directly to the thrusters to implement feed-forward control.
% However, in practice, a feedback controller is needed to reject
% disturbances and compensate for modeling errors.
%
% You can use different feedback control techniques for tracking. In this
% example, you use another nonlinear MPC controller to move the robot to
% the final location. In this path tracking problem, you track references
% for all six states.
nlobj_tracking = nlmpc(nx,ny,nu);

%% 
% Use the same state function and its Jacobian function.
nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Jacobian.StateFcn;

%%
% For feedback control applications, reduce the computational effort by
% specifying shorter prediction and control horizons.
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

%% 
% The default cost function in nonlinear MPC is a standard quadratic cost
% function suitable for reference tracking and disturbance rejection. For
% tracking, the states have higher priority (larger penalty weights) than
% the MV moves.
nlobj_tracking.Weights.ManipulatedVariablesRate = [0.2 0.2 0.2 0.2];%0.2*ones(1,nu);
%Qr for the state, since we don't tracking body velocity
nlobj_tracking.Weights.OutputVariables = [50 50 50 0 0 0];%5*ones(1,nx);

%% 
% Set the same bounds for the thruster inputs.
nlobj_tracking.MV = struct('Min',{-30;0;-30;0},'Max',{30;30;30;30});
% bounds for the state
nlobj_tracking.States = struct('Min',{-0.5;0.0;-pi/5;-inf;-inf;-inf},'Max',{0.5;1.2;pi/5;inf;inf;inf});

%%
% Also, to reduce fuel consumption, it is clear that |u1| and |u2| cannot be
% positive at any time during the operation. Therefore, implement equality
% constraints such that |u(1)*u(2)| must be |0| for all prediction steps.
% Apply similar constraints for |u3| and |u4|.
%nlobj_tracking.Optimization.CustomEqConFcn = ...
    %@(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

%%
% Validate your prediction model and custom functions, and their Jacobians.
validateFcns(nlobj_tracking,x0,u0);

%% Nonlinear State Estimation
% In this example, only the three position states (x, y and angle) are
% measured. The velocity states are unmeasured and must be estimated. Use
% an extended Kalman filter (EKF) from Control System Toolbox(TM) for
% nonlinear state estimation.
%
% Because an EKF requires a discrete-time model, you use the trapezoidal
% rule to transition from x(k) to x(k+1), which requires the solution of
% |nx| nonlinear algebraic equations. For more information, open
% |FlyingRobotStateFcnDiscreteTime.m|.
%DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);

%%
% Measurement can help the EKF correct its state estimation. Only the first
% three states are measured.
%DMeasFcn = @(xk) xk(1:3);

%%
% Create the EKF, and indicate that the measurements have little noise.
%EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
%EKF.MeasurementNoise = 0.01;

%% Closed-Loop Simulation of Tracking Control
% Simulate the system for |32| steps with correct initial conditions.
Sim_time = 40; %in sec
Tsteps = Sim_time/nlobj_tracking.Ts;
sim_t_seq = (1:Tsteps) * nlobj_tracking.Ts;

xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

%%
% The reference signals are the optimal state trajectories computed at the
% planning stage. When passing these trajectories to the nonlinear MPC
% controller, the current and future trajectory is available for
% previewing.
Xref = rbt_body_ref_traj(sim_t_seq)';

%%
% Use |nlmpcmove| and |nlmpcmoveopt| command for closed-loop simulation.
hbar = waitbar(0,'Simulation Progress');
options = nlmpcmoveopt;
for k = 1:Tsteps
    % Obtain plant output measurements with sensor noise.
    %yk = xHistory(k,1:3)' + randn*0.01;
    % Correct state estimation based on the measurements.
    %xk = correct(EKF, yk);
    % Compute the control moves with reference previewing.
    %In this demo we're not using ekf, just use ode calcuated state as
    %input, assume we have the 'global sensor' to catch robot's full state
    x_observed = xHistory(k,:)';
    [uk,options] = nlmpcmove(nlobj_tracking,...
                             x_observed,...
                             lastMV,...
                             Xref(k:min(k+(nlobj_tracking.PredictionHorizon - 1),Tsteps),:),...
                             [],...
                             options);
    % Predict the state for the next step.
    %predict(EKF,uk,Ts);
    % Store the control move and update the last MV for the next step.
    uHistory(k,:) = uk';
    lastMV = uk;
    % Update the real plant states for the next step by solving the
    % continuous-time ODEs based on current states xk and input uk.
    ODEFUN = @(t,xk) rbt_state_fcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % Store the state values.
    xHistory(k+1,:) = YOUT(end,:);            
    % Update the status bar.
    waitbar(k/Tsteps, hbar);
end
close(hbar)

%% 
% Compare the planned and actual closed-loop trajectories.
%FlyingRobotPlotTracking(info,Ts,p,Tsteps,xHistory,uHistory);
state_arr = xHistory;
state_arr(end,:) = [];
state_ref_arr = Xref;
input_arr = uHistory;
t_arr = sim_t_seq;

% visualization & save the video
plot_rbt_state_input;
rbt_vis_anime('test',state_arr,input_arr,[0.4 0; -0.4 0],0.5,0.55,0.65)
