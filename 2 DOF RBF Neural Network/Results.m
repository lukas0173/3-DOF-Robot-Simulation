% This is a comparison between the desired and actual trajectories
figure('Position',[200 320 500 280]);
plot(x.Data(end/2:end),y.Data(end/2:end),'-r','LineWidth',2);  % Actual trajectory
hold on;
grid on;
axis([0 1.5 0 1.5])
plot(xd.Data(end/2:end),yd.Data(end/2:end),'--k','LineWidth',2);  % Desired trajectory
xlabel('x(meter)');     % x-axis label
ylabel('y(meter)');     % y-axis label
title('Actual vs Desired Trajectories');    % Graph title
legend('Actual','Desired');   % Add the legend
axis square % Make the figure square in shape


%This figure shows the actual trajectory in animation
figure('Position',[750 320 500 280]);
plot(x.Data(1),y.Data(1),'-r');
hold on;
grid on;
axis([-0.5 1.5 -0.5 1.5]);
xlabel('x(meter)');     % x-axis label
ylabel('y(meter)');     % y-axis label
title('Animated actual trajectory');   % Graph title
axis square


% Utilize MATLAB built-in function to show actual trajectory
comet(x.Data,y.Data);  
