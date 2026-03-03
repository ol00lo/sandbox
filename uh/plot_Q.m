function plot_Q(time_vec, QI, QPo, wcP, qI, qP, qPo)
    figure('Position',[100,100,900,500]);
    plot(time_vec, qI*day,  'c-', 'LineWidth',1.5); hold on;
    plot(time_vec, -qP*day,  'k--', 'LineWidth',1.5);
    plot(time_vec, -qPo*day, 'r-', 'LineWidth',1.5);

    xlabel('Time [days]');
    ylabel('Rate [m^3/day]');
    title('Well rates');
    legend('Injection rate q_I', ...
           'Total production rate q_P', ...
           'Oil production rate q_P^o', ...
           'Location','best');
    grid on;

    figure;
    plot(time_vec, wcP*100, 'm','LineWidth',1.5);
    xlabel('Time [days]'); ylabel('Water Cut [%]');
    title('Producer Water Cut'); grid on;

    figure;
    plot(time_vec, QI, 'k', time_vec, -QPo, 'r', 'LineWidth',1.5);
    xlabel('Time [days]'); ylabel('Cumulative Volume [m^3]');
    title('Producer Cumulative Production');
    legend('Injected Water','Produced Oil'); grid on;
end