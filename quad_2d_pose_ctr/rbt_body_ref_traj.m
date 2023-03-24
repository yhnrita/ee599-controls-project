function [ xdesired ] = rbt_body_ref_traj( t )
%generate the desired traj

x_com = zeros(1,length(t))+ 0.12*sin(t);
z_com = 0.75 + 0.15*sin(2*t+pi/4);
theta_com = zeros(1,length(t))+ pi/10*cos(1.2*t);

dx_com = zeros(1,length(t));
dz_com = zeros(1,length(t));
dtheta_com = zeros(1,length(t));


xdesired = [x_com;z_com;theta_com;dx_com;dz_com;dtheta_com];
end