time_ = out.STATES(:,1);

figure;
sgtitle('states');
subplot(3,2,1)
plot(time_,out.STATES(:,2));
title('x');
grid on;
subplot(3,2,3)
plot(time_,out.STATES(:,3));
title('y');
grid on;
subplot(3,2,5)
plot(time_,out.STATES(:,4));
title('z');
grid on;
subplot(3,2,2)
plot(time_,rad2deg(out.STATES(:,5)));
title('\phi');
grid on;
subplot(3,2,4)
plot(time_,rad2deg(out.STATES(:,6)));
title('\theta');
grid on;
subplot(3,2,6)
plot(time_,rad2deg(out.STATES(:,7)));
title('\psi');
grid on;

figure;
sgtitle('input');
subplot(2,2,1)
plot(time_,out.CONTROLS(:,2));
title('f_t (N)');
grid on;
subplot(2,2,2)
plot(time_,out.CONTROLS(:,3));
title('\tau_x (Nm)');
grid on;
subplot(2,2,3)
plot(time_,out.CONTROLS(:,4));
title('\tau_y (Nm)');
grid on;
subplot(2,2,4)
plot(time_,out.CONTROLS(:,5));
title('\tau_z (Nm)');
grid on;