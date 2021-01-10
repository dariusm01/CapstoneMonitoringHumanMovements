MeasuredData = readtable("SampleData.xlsx");

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500;

%% Prediction 

% x = Fx + w

F = [1 0 0 dt 0 0; 
     0 1 0 0 dt 0; 
     0 0 1 0 0 dt; 
     0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1];
 
Wk = 0; 
 
AngleSim = sim("RateGyroUsingQuaternions.slx");

% Outputs 57x1
phi = AngleSim.phi.signals.values;
theta = AngleSim.theta.signals.values;
psi = AngleSim.psi.signals.values;
phi_dot = AngleSim.phi_dot.signals.values;
theta_dot = AngleSim.theta_dot.signals.values;
psi_dot = AngleSim.psi_dot.signals.values;

%% Resampling to get 50x1
% resamples the input sequence, x, at 7/8 times the original sample rate
% 57*(7/8) = 49.8750 -> ceil(49.8750) = 50
phi = resample(phi,7,8);
theta = resample(theta,7,8);
psi = resample(psi,7,8);
phi_dot = resample(phi_dot,7,8);
theta_dot = resample(theta_dot,7,8);
psi_dot = resample(psi_dot,7,8);

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
Phi = phi(1); Theta = theta(1); Psi = psi(1); 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
PhiDot = phi_dot(1); ThetaDot = theta_dot(1); PsiDot = psi_dot(1); 

% State Matrix
Xk_1 = [Phi; Theta; Psi; PhiDot; ThetaDot; PsiDot]; 

% Covariance Matrix 

Pk_1 = eye(length(F))*500;

AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Qk = eye(length(F));

Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);

Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);

% Measurement noise
Rk = eye(size(Pk_1))*0.1;

H = eye(size(Pk_1));

I = eye(size(H));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];
PhiDotKalman = [];
ThetaDotKalman = [];
PsiDotKalman = [];

%% Need the standard deviation and residuals to evaluate the filter mathematically
PosPhiSTD = [];
PosThetaSTD = [];

SpeedPhiSTD = [];
SpeedThetaSTD = [];

ResidualPhi = [];
ResidualTheta = [];

ResidualPhiDot = [];
ResidualThetaDot = [];

for i = 1:length(time)

    % Prior
    Xkp = F*Xk_1 + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
    % the measurement function (H) converts the filter’s prior into a measurement
    
    zk = [AccelX(i); AccelY(i); AccelZ(i); phi_dot(i); theta_dot(i); psi_dot(i)];  
    
    %% Jacobian 
    H(1,1) = -sind(Xkp(1))*sind(Xkp(2));
    H(2,1) = cosd(Xkp(1));
    H(3,1) = -sind(Xkp(1))*cosd(Xkp(2));
    
    H(1,2) = cosd(Xkp(1))*cosd(Xkp(2));
    H(2,2) = 0;
    H(3,2) = -cosd(Xkp(1))*sind(Xkp(2));
    
    H(3,3) = 0;
    
    % Measurement Model 
    
    % |ax|     |     sin(θy)     |
    % |ay|  =  | -cos(θy)sin(θx) |
    % |az|     |  cos(θx)cos(θy) |
 
    % For gyro measurment model, I have already converted the (p,q,r) to
    % angular rates using simulink
    
    h_of_x = [sind(Xkp(2));
              -cosd(Xkp(2))*sind(Xkp(1));
              cosd(Xkp(1))*cosd(Xkp(2));
              Xkp(4);
              Xkp(5);
              Xkp(6)];
    
    % Innovation (Residual)
    yk = zk - h_of_x;
    
    % Kalman Gain  
    K = Pkp*H.'*(Sk^-1);
    
    % Posterior 
    Xk = Xkp + K*yk;
    
    % Covariance Update
    Pk = (I - K*H)*Pkp*(I - K*H).' + (K*Rk*K.');
    
    % Redefining for next iteration
    Xk_1 = Xk;
    
    Pk_1 = Pk;
    
    
    % Store for plotting
    PhiKalman = [PhiKalman; Xk(1)];
    ThetaKalman = [ThetaKalman; Xk(2)];
    PsiKalman = [PsiKalman; Xk(3)];
    PhiDotKalman = [PhiDotKalman; Xk(4)];
    ThetaDotKalman = [ThetaDotKalman; Xk(5)];
    PsiDotKalman = [PsiDotKalman; Xk(6)];
    
    PosPhiSTD = [PosPhiSTD; sqrt(Pk(1,1))];
    PosThetaSTD = [PosThetaSTD; sqrt(Pk(2,2))];
    SpeedPhiSTD = [SpeedPhiSTD; sqrt(Pk(4,4))];
    SpeedThetaSTD = [SpeedThetaSTD; sqrt(Pk(5,5))];

    ResidualPhi = [ResidualPhi; yk(1)];
    ResidualTheta = [ResidualTheta; yk(2)];
    ResidualPhiDot = [ResidualPhiDot; yk(4)];
    ResidualThetaDot = [ResidualThetaDot; yk(5)];
end 

%% Plotting

figure(1)
plot(time, phi_dot)
title('Roll Angle Rate $\dot{\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, PhiDotKalman)
legend("Measured Gyro data \phi", "Extended Kalman Filter Gyro data \phi")
hold off

figure(2)
plot(time, theta_dot)
title('Pitch Angle Rate $\dot{\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, ThetaDotKalman)
legend("Measured Gyro data \theta", "Extended Kalman Filter Gyro \theta")
hold off

figure(3)
plot(time, psi_dot)
title('Yaw Angle Rate $\dot{\psi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, PsiDotKalman)
legend("Measured Gyro data \psi", "Extended Kalman Filter Gyro \psi")
hold off


figure(4)
plot(time, PhiKalman)
title('Roll Angle ${\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, phi)
legend("Extended Kalman Filter \phi", "Gyroscope \phi")
% legend("Extended Kalman Filter \Phi")
hold off

figure(5)
plot(time, ThetaKalman)
title('Pitch Angle ${\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, theta)
legend("Extended Kalman Filter \theta", "Gyroscope \theta")
%legend("Extended Kalman Filter \Theta")
hold off

% Plotting Residuals
figure(6)
plot(time, 3*PosPhiSTD, 'ko')
title("Roll Angle Residuals 3\sigma")
ylabel("Degrees [°]")
grid on
hold on
plot(time, ResidualPhi)
plot(time, -3*PosPhiSTD, 'ko')
hold off

figure(7)
plot(time, 3*PosThetaSTD, 'ko')
title("Pitch Angle Residuals 3\sigma")
ylabel("Degrees [°]")
grid on
hold on
plot(time, ResidualTheta)
plot(time, -3*PosThetaSTD, 'ko')
hold off

figure(8)
plot(time, 3*SpeedPhiSTD, 'ko')
title("Roll Rate Residuals 3\sigma")
ylabel("Degrees [°/s]")
grid on
hold on
plot(time, ResidualPhiDot)
plot(time, -3*SpeedPhiSTD, 'ko')
hold off

figure(9)
plot(time, 3*SpeedThetaSTD, 'ko')
title("Pitch Rate Residuals 3\sigma")
ylabel("Degrees [°/s]")
grid on
hold on
plot(time, ResidualThetaDot)
plot(time, -3*SpeedThetaSTD, 'ko')
hold off

