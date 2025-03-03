function plot_staticEM(k, psih, e)
    % Colours
    blue = '#0072BD';
    lblue = '#4DBEEE';
    orange = '#ED872D';
    % Plot
    figure
    stairs(k, psih(1, :), 'Color', blue, 'LineWidth', 4)
    hold on
    stairs(k, psih(2, :), 'Color', lblue, 'LineWidth', 4)
    stairs(k, e, 'Color', orange, 'LineWidth', 4)
    hold off
    legend({'$\hat{\psi}_1$', '$\hat{\psi}_2$', '$e$'}, 'Interpreter','latex', 'FontSize', 14)
    grid on
end