figure;
hold on;

plot(out.STATES(:,1), out.STATES(:,2), 'r')
plot(out.STATES(:,1), 180/3.14*out.STATES(:,3), 'b')
hold off;
legend('x', 'alpha');
grid on;

figure
plot(out.CONTROLS(:,1), out.CONTROLS(:,2))
legend('F');
grid on;