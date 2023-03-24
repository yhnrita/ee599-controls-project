% Plot the closed-loop response.

% Plot the states.
clf;

%figure('Name','States')

subplot(6,2,1)
hold on
plot(t_arr,state_arr(:,1),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('m')
%legend('actual','reference','Location','southeast')
title('x_{com} pos')

subplot(6,2,2)
hold on
plot(t_arr,state_arr(:,4),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('m/s')
%legend('actual','reference','Location','southeast')
title('x_{com} vel')

subplot(6,2,3)
hold on
plot(t_arr,state_arr(:,2),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('m')
%legend('actual','reference','Location','southeast')
title('z_{com} pos')

subplot(6,2,4)
hold on
plot(t_arr,state_arr(:,5),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('m/s')
%legend('actual','reference','Location','southeast')
title('z_{com} vel')

subplot(6,2,5)
hold on
plot(t_arr,state_arr(:,3),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('rad')
%legend('actual','reference','Location','southeast')
title('theta')

subplot(6,2,6)
hold on
plot(t_arr,state_arr(:,6),'linewidth',1.2)
grid on
xlabel('time - s')
ylabel('rad/s')
%legend('actual','reference','Location','southeast')
title('theta vel')

subplot(6,2,7)
hold on
plot(t_arr,input_arr(:,1),'linewidth',1.2,'color','red')
grid on
xlabel('time - s')
ylabel('N')
%legend('actual','reference','Location','southeast')
title('leg f force x')

subplot(6,2,8)
hold on
plot(t_arr,input_arr(:,2),'linewidth',1.2,'color','red')
grid on
xlabel('time - s')
ylabel('N')
%legend('actual','reference','Location','southeast')
title('leg f force z')

subplot(6,2,9)
hold on
plot(t_arr,input_arr(:,3),'linewidth',1.2,'color','red')
grid on
xlabel('time - s')
ylabel('N')
%legend('actual','reference','Location','southeast')
title('leg b force x')

subplot(6,2,10)
hold on
plot(t_arr,input_arr(:,4),'linewidth',1.2,'color','red')
grid on
xlabel('time - s')
ylabel('N')
%legend('actual','reference','Location','southeast')
title('leg b force z')
