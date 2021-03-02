mat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/NED_Frame/x_AxisRotation.xlsx");

GyroX = mat.GyroX;
GyroY = mat.GyroY;
GyroZ = mat.GyroZ;

simTime = length(GyroX) - 1;

time = 1:simTime + 1;

time = time.';

h = 1/100;

Eulers = sim("RateGyroUsingQuaternions.slx");

%% Gathering Euler Angles

Phi = Eulers.phi.signals.values;
Theta = Eulers.theta.signals.values;
Psi = Eulers.psi.signals.values;

PhiDot = Eulers.phi_dot.signals.values;
ThetaDot = Eulers.theta_dot.signals.values;
PsiDot = Eulers.psi_dot.signals.values;

T = table(Phi,Theta,Psi,PhiDot,ThetaDot,PsiDot);

writetable(T,"/Users/dariusmensah/Desktop/LabTesting/MPU_6050/NED_Frame/x_AxisEulerAngles.xlsx");

clearvars;
clc;