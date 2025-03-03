function plot_dynamicEM(k, psih, xe)
    % Colours
    blue = '#0072BD';
    lblue = '#4DBEEE';
    orange = '#ED872D';
    yellow = '#EDB120';
    % Plot
    figure
    stairs(k, psih(1, :), 'Color', blue, 'LineWidth', 4)
    hold on
    stairs(k, psih(2, :), 'Color', lblue, 'LineWidth', 4)
    stairs(k, xe(1, :), 'Color', orange, 'LineWidth', 4)
    stairs(k, xe(2, :), 'Color', yellow, 'LineWidth', 4)
    hold off
    legend({'$\hat{\psi}_1$', '$\hat{\psi}_2$', '$(x_e)_1$', '$(x_e)_2$'},'Interpreter', 'latex', 'FontSize', 14)
    grid on
end