fig = figure;
fig.Position(3:4) = [800 400];

plot(states.Time, states.Data(:, 3), 'LineWidth', 1.5);
grid on;

xlabel('Time [s]', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('Amplitude $\theta$ [rad]', 'FontSize', 18, 'Interpreter', 'latex');

set(gca, 'FontSize', 16);
