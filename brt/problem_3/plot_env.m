function plots = plot_env(traj, params)

figure(1);
clf;
box off;
grid off;
x0 = 10;
y0 = 10;
width = 800;
length = 800;
set(gcf,'position',[x0,y0,width,length])
view(3);

%% 3D Obstacles Visualization
% Plot cuboid 1 in 3D
PlotCuboid(params.obsX1, params.obsY1, params.obsZ1, params.obslength1, params.obswidth1, params.obsheight1);
% text(params.obsX1 + (params.obslength1)/2,params.obsY2 - (params.obswidth1)/5,'B1','FontSize',18);

% hold on;

% Plot cuboid 2 in 3D
% PlotCuboid(params.obsX2, params.obsY2, params.obsZ2, params.obslength2, params.obswidth2, params.obsheight2);
% text(params.obsX2 + (params.obslength2)/2,params.obsY2 - (params.obswidth2)/5,'B2','FontSize',18);

% rectangle('Position',[params.obsX1 params.obsY1 params.obslength1 params.obswidth1 ],...
%     'EdgeColor','r','FaceColor',[1 0 0],'Curvature',0.1);


hold on;
text(params.obsX1-(params.obslength1)/5,params.obsY2+(params.obswidth1)/2,'Ground','FontSize',18);


xlim([-10, 10]);
ylim([-10, 10]);
zlim([-10, 10]);
set(gca,'XTickLabel',[]);
set(gca,'YTickLabel',[]);
set(gca,'ZTickLabel',[]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory');
fig = gcf;
fig.Name = 'BRT in 3D';
fig.Color = [1 1 1];
fig.NumberTitle = 'off';
view([-63 11]);


%% 2D Obstacles Visualization
% Create obstacle 1
% PlotCuboid([5,10,15],[2,3,4]);
rectangle('Position',[params.obsX1 params.obsY1 params.obslength1 params.obswidth1 ],...
    'EdgeColor','g','FaceColor', 'g','Curvature',0.1);
 hold on;
% text(params.obsX1+(params.obslength1)/2,params.obsY2+(params.obswidth1)/2,'B1','FontSize',18);

% Create obstacle 2
% rectangle('Position',[params.obscenX-params.obsR params.obscenY-params.obsR ...
%     2*params.obsR 2*params.obsR],...
%     'EdgeColor','r','FaceColor',[1 0 0],'Curvature',[1 1]);
% text(params.obscenX,params.obscenY,'B2','FontSize',18)

% 
% Create obstacle 2
% rectangle('Position',[params.obsX2 params.obsY2 params.obslength2 params.obswidth2],...
%     'EdgeColor','r','FaceColor',[1 0 0],'Curvature',0.1);
% hold on;
% text(params.obsX2+(params.obslength2)/2,params.obsY2+(params.obswidth2)/2,'B2','FontSize',18);

% Create goal point
% viscircles([params.goalX, params.goalY], [params.goalR], 'color', 'g');
% rectangle('Position',[params.goalX - params.goalR params.goalY - params.goalR ...
%     2*params.goalR 2*params.goalR],...
%     'EdgeColor','k','FaceColor',[0 1 0],'Curvature',[1 1]);

% Create goal point
[X_tmp,Y_tmp,Z_tmp] = sphere;
r = 0.2;
X_plot = X_tmp * r;
Y_plot = Y_tmp * r;
Z_plot = Z_tmp * r;
surf(X_plot + params.goalX, Y_plot + params.goalY, Z_plot + params.goalZ);
text(params.goalX - 3,params.goalY + 1,'Goal','FontSize',16)

if ~isempty(traj)
    % initial state
    xinit = traj(:,1);
    % Create initial state
%     viscircles(xinit(1:2,:)', [0.1], 'color', 'b');
    [X_tmp,Y_tmp,Z_tmp] = sphere;
    r = 0.2;
    X_plot = X_tmp * r;
    Y_plot = Y_tmp * r;
    Z_plot = Z_tmp * r;
    surf(X_plot + params.xinit(1), Y_plot + params.xinit(2), Z_plot + params.xinit(3));
    %text(params.xinit(1),params.xinit(2) - 3,'Start','FontSize',16)

%     plot(traj(1, :), traj(2, :), 'color', 'b', 'LineWidth', 2);
    plot3(traj(1, :), traj(2, :), traj(3, :), 'color', 'b', 'LineWidth', 2);
    
end





axis square;
box on
plots = 0;


function PlotCuboid(X, Y, Z, length, width, height, color)
%% Function function: Draw a box
% Input:
%       X, Y, Z: the origin of the box, row vector, such as [0, 0, 0];
%       length, width, height: length, width and height of the box, row vector, such as [10, 20, 30];
%       color: fill each facet with color if you like
% Output: box shapes

%% Define the vertices corresponding to each of the 6 planes
x = [X X + length X + length X X X X X X X X X + length X + length X + length X + length X + length X + length X + length X];
y = [Y Y Y Y Y Y Y + width Y + width Y Y + width Y + width Y + width Y + width Y Y Y + width Y + width Y + width Y + width];
z = [Z Z Z + height Z + height Z Z + height Z + height Z Z Z Z + height Z + height Z Z Z + height Z + height Z Z Z];
plot3(x,y,z,'r');

if nargin == 7
    %% Fill each planes with colors
    fill3([X X + length X + length X], [Y Y Y Y], [Z Z Z + height Z + height], color);
    fill3([X X X X ], [Y Y + width Y + width Y], [Z + height Z + height Z Z], color);
    fill3([X X X + length X + length ], [Y + width Y + width Y + width Y + width], [Z Z + height Z + height Z], color);
    fill3([X + length X + length X + length X + length], [Y Y Y + width Y + width], [Z Z + height Z + height Z], color);
    fill3([X X X + length X + length ], [Y Y + width Y + width Y], [Z + height Z + height Z + height Z + height], color);
    fill3([X X X + length X + length ], [Y Y + width Y + width Y], [Z Z Z Z], color);
end

function orbit(deg)
[az, el] = view;
rotvec = 0:deg/10:deg;
for i = 1:length(rotvec)
    view([az+rotvec(i) el])
    drawnow
end

