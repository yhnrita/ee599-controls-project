function [] = rbt_vis_anime(saved_video_name, state_arr, input_arr, foot_pos, l1, l2, l3)

x_com_p = state_arr(:,1);
z_com_p = state_arr(:,2);
theta_p = state_arr(:,3);

frame_num = size(x_com_p,1);

%figure('Name','Robot vis');
subplot(6,2,11);
hold on;

b_c_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',2,'EdgeColor','red','FaceColor',[1 1 1]);
b_f_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',2,'EdgeColor','blue','FaceColor',[1 1 1]);
b_b_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',2,'EdgeColor','blue','FaceColor',[1 1 1]);

knee_f_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1.2,'EdgeColor','blue','FaceColor',[1 1 1]);
knee_b_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1.2,'EdgeColor','blue','FaceColor',[1 1 1]);

hip_f_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1.2,'EdgeColor','blue','FaceColor',[1 1 1]);
hip_b_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1.2,'EdgeColor','blue','FaceColor',[1 1 1]);

force_f_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1,'EdgeColor','m','FaceColor',[1 1 1]);
force_b_vis = patch([0 0],[0 1],[1 1 1],'LineWidth',1,'EdgeColor','m','FaceColor',[1 1 1]);

b_f_p_vis = [x_com_p+l1*cos(theta_p), z_com_p-l1*sin(theta_p)]; %frame_num * 2
b_b_p_vis = [x_com_p-l1*cos(theta_p), z_com_p+l1*sin(theta_p)]; 

leg_vec_f = b_f_p_vis - foot_pos(1,:);
leg_vec_b = b_b_p_vis - foot_pos(2,:);

ang_arr_1_f = atan2(leg_vec_f(:,2),leg_vec_f(:,1));
ang_arr_1_b = atan2(leg_vec_b(:,2),leg_vec_b(:,1));

lvec_l_pow_f = leg_vec_f(:,1).*leg_vec_f(:,1) + leg_vec_f(:,2).*leg_vec_f(:,2);
lvec_l_pow_b = leg_vec_b(:,1).*leg_vec_b(:,1) + leg_vec_b(:,2).*leg_vec_b(:,2);

ang_arr_2_f = acos((l3*l3+lvec_l_pow_f-l2*l2) ./ (2*l3.*sqrt(lvec_l_pow_f)));
ang_arr_2_b = acos((l3*l3+lvec_l_pow_b-l2*l2) ./ (2*l3.*sqrt(lvec_l_pow_b)));

ang_arr_3_f = ang_arr_1_f + ang_arr_2_f;
ang_arr_3_b = ang_arr_1_b + ang_arr_2_b;

knee_f_p_vis = [foot_pos(1,1)+l3*cos(ang_arr_3_f), foot_pos(1,2)+l3*sin(ang_arr_3_f)];
knee_b_p_vis = [foot_pos(2,1)+l3*cos(ang_arr_3_b), foot_pos(2,2)+l3*sin(ang_arr_3_b)];

force_vis_k = 0.05;
force_f = [input_arr(:,1), input_arr(:,2)] * force_vis_k; %force front fx,fz
force_b = [input_arr(:,3), input_arr(:,4)] * force_vis_k; 

axis equal;
xlim([-1.5,1.5]);
ylim([0,1.5]);

    window_pos = get(gcf, 'Position');
    set(gcf, 'Position',window_pos+[0 -400 0 400]);
    
v = VideoWriter('sim_p.avi');
open(v);

for f_i = 1:frame_num
    
    b_c_vis.XData = [x_com_p(f_i), x_com_p(f_i)+0.05*cos(0:0.05:2*pi)];
    b_c_vis.YData = [z_com_p(f_i), z_com_p(f_i)+0.05*sin(0:0.05:2*pi)];
    
    b_f_vis.XData = [x_com_p(f_i), b_f_p_vis(f_i,1)];
    b_f_vis.YData = [z_com_p(f_i), b_f_p_vis(f_i,2)];
    b_b_vis.XData = [x_com_p(f_i), b_b_p_vis(f_i,1)];
    b_b_vis.YData = [z_com_p(f_i), b_b_p_vis(f_i,2)];
    
    knee_f_vis.XData = [foot_pos(1,1), knee_f_p_vis(f_i,1), knee_f_p_vis(f_i,1)+0.03*cos(0:0.05:2*pi), knee_f_p_vis(f_i,1)];
    knee_f_vis.YData = [foot_pos(1,2), knee_f_p_vis(f_i,2), knee_f_p_vis(f_i,2)+0.03*sin(0:0.05:2*pi), knee_f_p_vis(f_i,2)];
    
    knee_b_vis.XData = [foot_pos(2,1), knee_b_p_vis(f_i,1), knee_b_p_vis(f_i,1)+0.03*cos(0:0.05:2*pi), knee_b_p_vis(f_i,1)];
    knee_b_vis.YData = [foot_pos(2,2), knee_b_p_vis(f_i,2), knee_b_p_vis(f_i,2)+0.03*sin(0:0.05:2*pi), knee_b_p_vis(f_i,2)];
    
    hip_f_vis.XData = [knee_f_p_vis(f_i,1), b_f_p_vis(f_i,1), b_f_p_vis(f_i,1)+0.03*cos(0:0.05:2*pi), b_f_p_vis(f_i,1)];
    hip_f_vis.YData = [knee_f_p_vis(f_i,2), b_f_p_vis(f_i,2), b_f_p_vis(f_i,2)+0.03*sin(0:0.05:2*pi), b_f_p_vis(f_i,2)];
    hip_b_vis.XData = [knee_b_p_vis(f_i,1), b_b_p_vis(f_i,1), b_b_p_vis(f_i,1)+0.03*cos(0:0.05:2*pi), b_b_p_vis(f_i,1)];
    hip_b_vis.YData = [knee_b_p_vis(f_i,2), b_b_p_vis(f_i,2), b_b_p_vis(f_i,2)+0.03*sin(0:0.05:2*pi), b_b_p_vis(f_i,2)];
    
    %draw the force
    force_f_vis.XData = [foot_pos(1,1), foot_pos(1,1) + force_f(f_i,1)];
    force_f_vis.YData = [foot_pos(1,2), foot_pos(1,2) + force_f(f_i,2)];
    force_b_vis.XData = [foot_pos(2,1), foot_pos(2,1) + force_b(f_i,1)];
    force_b_vis.YData = [foot_pos(2,2), foot_pos(2,2) + force_b(f_i,2)];
    
    frame = getframe(gcf);
    writeVideo(v,frame);
   
    %pause(1/24);
    
    drawnow;
end

close(v);

end

