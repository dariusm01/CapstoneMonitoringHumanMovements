MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500; 

open("RateGyro_to_EulerAngles.slx")
AngleSim = sim("RateGyro_to_EulerAngles.slx");

phi = AngleSim.phi.signals.values;
theta = AngleSim.theta.signals.values;
psi = AngleSim.psi.signals.values;
t = AngleSim.phi.time;
phi_dot = AngleSim.phi_dot.signals.values;
theta_dot = AngleSim.theta_dot.signals.values;
psi_dot = AngleSim.psi_dot.signals.values;

%% Angular Position
figure(1)
plot(t, phi)
grid on
title("Roll Angle \phi")
ylabel("Degrees [°]")

figure(2)
plot(t, theta)
grid on
title("Pitch Angle \theta")
ylabel("Degrees [°]")

figure(3)
plot(t, psi)
grid on
title("Yaw Angle \psi")
ylabel("Degrees [°]")

%% Angular Velocity
figure(4)
plot(t, phi_dot)
grid on
ylabel("Degrees Per Second [°/s]")
title('Roll Angle Rate $\dot{\phi}$','interpreter','latex')

figure(5)
plot(t, theta_dot)
grid on
ylabel("Degrees Per Second [°/s]")
title('Pitch Angle Rate $\dot{\theta}$','interpreter','latex')

figure(6)
plot(t, psi_dot)
grid on
ylabel("Degrees Per Second [°/s]")
title('Yaw Angle Rate $\dot{\psi}$','interpreter','latex')





