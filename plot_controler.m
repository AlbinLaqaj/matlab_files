txt = { ...
    '$\phi_1$', ...
    '$\phi_2$', ...
    '$\theta$', ...
    '$\dot{\phi}_1$', ...
    '$\dot{\phi}_2$', ...
    '$\dot{\theta}$' ...
};

% % Plot phi1, phi2
% fig1 = figure(1);
% fig1.Position(3:4) = [800 600];
% 
% for i = 1:2
%     subplot(2, 1, i);
%     plot(y1.Time, y1.Data(:, i), 'b', 'LineWidth', 1.5);
%     hold on;
%     plot(y2.Time, y2.Data(:, i), 'r', 'LineWidth', 1.5); 
%     hold off;
% 
%     grid on;
%     xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
%     ylabel(['Amplitude ', txt{i}, ' [deg]'], 'FontSize', 18, 'Interpreter', 'latex');
%     legend('PID', 'LQR', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
%     set(gca, 'FontSize', 16);
% end
% 
% % Plot phi1_dot, phi2_dot
% fig2 = figure(2);
% fig2.Position(3:4) = [800 600];
% 
% for i = 4:5
%     subplot(2, 1, i - 3);
%     plot(y1.Time, y1.Data(:, i), 'b', 'LineWidth', 1.5);
%     hold on;
%     plot(y2.Time, y2.Data(:, i), 'r', 'LineWidth', 1.5);
%     hold off;
% 
%     grid on;
%     xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
%     ylabel(['Amplitude ', txt{i}, ' [deg/s]'], 'FontSize', 18, 'Interpreter', 'latex');
%     legend('PID', 'LQR', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
%     set(gca, 'FontSize', 16);
% end

% Plot theta
fig3 = figure(3);
fig3.Position(3:4) = [800 500];

plot(y1.Time, y1.Data(:, 3), 'b', 'LineWidth', 1.5);
hold on;
plot(y2.Time, y2.Data(:, 3), 'r', 'LineWidth', 1.5);
hold off;

grid on;
xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
ylabel(['Amplitude ', txt{3}, ' [deg]'], 'FontSize', 18, 'Interpreter', 'latex');
legend('PID', 'LQR', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
set(gca, 'FontSize', 16);
yticks(-5:2.5:17.5);

% % Plot theta_dot
% fig4 = figure(4);
% fig4.Position(3:4) = [800 500];
% 
% plot(y1.Time, y1.Data(:, 6), 'b', 'LineWidth', 1.5);
% hold on;
% plot(y2.Time, y2.Data(:, 6), 'r', 'LineWidth', 1.5);
% hold off;
% 
% grid on;
% xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
% ylabel(['Amplitude ', txt{6}, ' [deg/s]'], 'FontSize', 18, 'Interpreter', 'latex');
% legend('PID', 'LQR', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
% set(gca, 'FontSize', 16);

% Plot theta and voltage PID
fig5 = figure(5);
fig5.Position(3:4) = [800 500];

plot(y1.Time, y1.Data(:, 3), 'b', 'LineWidth', 1.5);
hold on;
plot(y1.Time, y1.Data(:, 7), 'r', 'LineWidth', 1.5);
hold off;

grid on;
xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
ylabel(['Amplitude ', txt{3}, ' [deg]', ', Voltage $U$ [V]'], 'FontSize', 18, 'Interpreter', 'latex');
legend(txt{3}, '$U$', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
set(gca, 'FontSize', 16);
yticks(-5:2.5:17.5);

% Plot theta and voltage LQR
fig6 = figure(6);
fig6.Position(3:4) = [800 500];

plot(y2.Time, y2.Data(:, 3), 'b', 'LineWidth', 1.5);
hold on;
plot(y2.Time, y2.Data(:, 7), 'r', 'LineWidth', 1.5);
hold off;

grid on;
xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
ylabel(['Amplitude ', txt{3}, ' [deg]', ', Voltage $U$ [V]'], 'FontSize', 18, 'Interpreter', 'latex');
legend(txt{3}, '$U$', 'FontSize', 16, 'Interpreter', 'latex', 'Location', 'northeast');
set(gca, 'FontSize', 16);
yticks(-5:2.5:17.5);
