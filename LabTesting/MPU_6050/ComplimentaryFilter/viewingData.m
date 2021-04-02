mat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/ComplimentaryFilter/y_AxisRotationFast.xlsx");

Phi = mat.Phi;
Theta = mat.Theta;

time = 1:length(Phi);
time = time.' * (1/100);

figure(1)
plot(time,Phi, 'LineWidth',1)
grid on
title("Rotation about X-Axis (Roll)")
ylabel("Degrees [°]")
xlabel("Time [s]")


figure(2)
plot(time,Theta,'LineWidth',1)
grid on
title("Rotation about Y-Axis (Pitch)")
ylabel("Degrees [°]")
xlabel("Time [s]")