figure

subplot(2,1,1)
plot(out.STATES(:,1),out.STATES(:,2));
legend('x');

subplot(2,1,2)
plot(out.STATES(:,1),out.STATES(:,3));
legend('xdot');

figure
plot(out.CONTROLS(:,1),out.CONTROLS(:,2));