load("MPU6050_newSamples.mat")

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

dt = 1/100;

%% Prediction 

% x = Fx + w

F = [1 0 0 dt 0 0 0.5*dt^2 0 0; 
     0 1 0 0 dt 0 0 0.5*dt^2 0; 
     0 0 1 0 0 dt 0 0 0.5*dt^2; 
     0 0 0 1 0 0 dt 0 0; 
     0 0 0 0 1 0 0 dt 0; 
     0 0 0 0 0 1 0 0 dt;
     0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 1];
 
% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
PhiDot = phi_dot(1); ThetaDot = theta_dot(1); PsiDot = psi_dot(1); 

PhiDD = 0; ThetaDD = 0; PsiDD = 0;

% State Matrix
Xk_1 = [phi(1); theta(1); psi(1); PhiDot; ThetaDot; PsiDot; PhiDD; ThetaDD; PsiDD]; 

% Covariance Matrix 
Pk_1 = eye(length(F))*500;

% Noise
AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Wk = 0;

Qk = SecondOrderPNC(100,dt);

% Measurement noise

H = zeros(6,length(Xk_1));
H(1,1) = 1;
H(2,2) = 1;
H(3,3) = 1;
H(4,4) = 1;
H(5,5) = 1;
H(6,6) = 1;

Rk = eye(6)*0.3;

I = eye(length(F));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];
PhiDotKalman = [];
ThetaDotKalman = [];
PsiDotKalman = [];

% Euler Angle Acceleration
PhiDDotKalman = [];
ThetaDDotKalman = [];
PsiDDotKalman = [];

% Sensor was not moving so everything should be close to 0
True = 0;

%% Need the standard deviation and residuals to evaluate the filter mathematically
PosPhiSTD = [];
PosThetaSTD = [];

SpeedPhiSTD = [];
SpeedThetaSTD = [];

AccelPhiSTD = [];
AccelThetaSTD = [];

ResidualPhi = [];
ResidualTheta = [];

ResidualPhiDot = [];
ResidualThetaDot = [];

ResidualPhiDDot = [];
ResidualThetaDDot = [];

for i = 1:length(time)

    % Prior
    Xkp = F*Xk_1 + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
 
    zk = [AccelX(i); AccelY(i); AccelZ(i); GyroX(i); GyroY(i); GyroZ(i)]; 
    
    % Measurement Model Accelerometer
    
    % |ax|     |    -sin(θ)    |
    % |ay|  =  |  cos(θ)sin(φ) |
    % |az|     |  cos(θ)cos(φ) |
 
    % Measurement Model Gyroscope
    %                   .   .
    % |p|     |         φ - ψsin(θ)      |
    %           .         .
    % |q|  =  | θcos(φ) + ψcos(θ)sin(φ)  |
    %           .               .
    % |r|     | ψcos(θ)cos(φ) - θsin(φ)  |
    
    % the measurement function converts the filter’s prior into a measurement
    h_of_x = [-sin(Xkp(2));
              cos(Xkp(2))*sin(Xkp(1));
              cos(Xkp(2))*cos(Xkp(1));
              Xkp(4)-(Xkp(6)*sin(Xkp(2)));
              Xkp(5)*cos(Xkp(1)) + Xkp(6)*cos(Xkp(2))*sin(Xkp(1));
              Xkp(6)*cos(Xkp(2))*cos(Xkp(1)) - Xkp(5)*sin(Xkp(1))];
    
          
    %% Measurment Jacobian 
    H(1,1) = 0; H(1,2) = -cos(Xkp(2)); 
    H(2,1) = cos(Xkp(1))*cos(Xkp(2)); H(2,2) = -sin(Xkp(1))*sin(Xkp(2));
    H(3,1) = -cos(Xkp(2))*sin(Xkp(1)); H(3,2) = -cos(Xkp(1))*sin(Xkp(2)); H(3,3) = 0;
    H(4,2) = -Xkp(6)*cos(Xkp(2)); H(4,6) = -sin(Xkp(2));
    H(5,1) = Xkp(6)*cos(Xkp(1))*cos(Xkp(2));
    H(5,2) = -Xkp(6)*sin(Xkp(1))*sin(Xkp(2));
    H(5,3) = -Xkp(5)*sin(Xkp(3));
    H(5,5) = cos(Xkp(3));
    H(5,6) = cos(Xkp(2))*sin(Xkp(3));
    H(6,1) = -Xkp(5)*cos(Xkp(1)) - Xkp(6)*cos(Xkp(2))*sin(Xkp(1));
    H(6,2) = -Xkp(6)*cos(Xkp(1))*sin(Xkp(2));
    H(6,5) = -sin(Xkp(1));
    H(6,6) = cos(Xkp(1))*cos(Xkp(2));
    
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
    PhiDDotKalman = [PhiDDotKalman; Xk(7)];
    ThetaDDotKalman = [ThetaDDotKalman; Xk(8)];
    PsiDDotKalman = [PsiDDotKalman; Xk(9)];
    
    PosPhiSTD = [PosPhiSTD; sqrt(Pk(1,1))];
    PosThetaSTD = [PosThetaSTD; sqrt(Pk(2,2))];
    SpeedPhiSTD = [SpeedPhiSTD; sqrt(Pk(4,4))];
    SpeedThetaSTD = [SpeedThetaSTD; sqrt(Pk(5,5))];
    AccelPhiSTD = [AccelPhiSTD; sqrt(Pk(7,7))];
    AccelThetaSTD = [AccelThetaSTD; sqrt(Pk(8,8))];

    ResidualPhi = [ResidualPhi; True-Xk(1)];
    ResidualTheta = [ResidualTheta; True-Xk(2)];
    ResidualPhiDot = [ResidualPhiDot; True-Xk(4)];
    ResidualThetaDot = [ResidualThetaDot; True-Xk(5)];
    ResidualPhiDDot = [ResidualPhiDDot; True-Xk(7)];
    ResidualThetaDDot = [ResidualThetaDDot; True-Xk(8)];
end 

%% Plotting

figure(1)
plot(time, rad2deg(phi_dot),'LineWidth',1)
title('Roll Angle Rate $\dot{\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(PhiDotKalman),'LineWidth',1)
legend("Measured Gyro data \phi", "Extended Kalman Filter Gyro data \phi")
hold off

figure(2)
plot(time, rad2deg(theta_dot),'LineWidth',1)
title('Pitch Angle Rate $\dot{\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(ThetaDotKalman),'LineWidth',1)
legend("Measured Gyro data \theta", "Extended Kalman Filter Gyro \theta")
hold off

figure(3)
plot(time, rad2deg(psi_dot),'LineWidth',1)
title('Yaw Angle Rate $\dot{\psi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(PsiDotKalman),'LineWidth',1)
legend("Measured Gyro data \psi", "Extended Kalman Filter Gyro \psi")
hold off


figure(4)
plot(time, rad2deg(PhiKalman),'LineWidth',1)
title('Roll Angle ${\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(phi),'LineWidth',1)
legend("Extended Kalman Filter \phi", "Gyroscope \phi")
% legend("Extended Kalman Filter \Phi")
hold off

figure(5)
plot(time, rad2deg(ThetaKalman),'LineWidth',1)
title('Pitch Angle ${\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(theta),'LineWidth',1)
legend("Extended Kalman Filter \theta", "Gyroscope \theta")
%legend("Extended Kalman Filter \Theta")
hold off

% Plotting Residuals
figure(6)
plot(time, 3*rad2deg(PosPhiSTD), 'ko')
title("Roll Angle Residuals 3\sigma")
ylabel("Degrees [°]")
grid on
hold on
plot(time, rad2deg(ResidualPhi))
plot(time, -3*rad2deg(PosPhiSTD), 'ko')
hold off

figure(7)
plot(time, 3*rad2deg(PosThetaSTD), 'ko')
title("Pitch Angle Residuals 3\sigma")
ylabel("Degrees [°]")
grid on
hold on
plot(time, rad2deg(ResidualTheta))
plot(time, -3*rad2deg(PosThetaSTD), 'ko')
hold off

figure(8)
plot(time, 3*rad2deg(SpeedPhiSTD), 'ko')
title("Roll Rate Residuals 3\sigma")
ylabel("Degrees [°/s]")
grid on
hold on
plot(time, rad2deg(ResidualPhiDot))
plot(time, -3*rad2deg(SpeedPhiSTD), 'ko')
hold off

figure(9)
plot(time, 3*rad2deg(SpeedThetaSTD), 'ko')
title("Pitch Rate Residuals 3\sigma")
ylabel("Degrees [°/s]")
grid on
hold on
plot(time, rad2deg(ResidualThetaDot))
plot(time, -3*rad2deg(SpeedThetaSTD), 'ko')
hold off

% figure(10)
% plot(time, 3*rad2deg(AccelPhiSTD), 'ko')
% title("Pitch Acceleration Residuals 3\sigma")
% ylabel("Degrees [°/s^2]")
% grid on
% hold on
% plot(time, rad2deg(ResidualPhiDDot))
% plot(time, -3*rad2deg(AccelPhiSTD), 'ko')
% hold off
% 
% figure(11)
% plot(time, 3*rad2deg(AccelThetaSTD), 'ko')
% title("Pitch Acceleration Residuals 3\sigma")
% ylabel("Degrees [°/s^2]")
% grid on
% hold on
% plot(time, rad2deg(ResidualThetaDDot))
% plot(time, -3*rad2deg(AccelThetaSTD), 'ko')
% hold off

%% Plotting Derivatives

figure(12)
subplot(3,1,1)
plot(time, rad2deg(PhiKalman))
grid on
ylabel("Degrees [°]")
title("Angular Position")

subplot(3,1,2)
plot(time, rad2deg(PhiDotKalman))
grid on
ylabel("Degrees [°/s]")
title("Angular Velocity")

subplot(3,1,3)
plot(time, rad2deg(PhiDDotKalman))
ylabel("Degrees [°/s^2]")
title("Angular Acceleration")
grid on

figure(13)
subplot(3,1,1)
plot(time, rad2deg(ThetaKalman))
grid on
ylabel("Degrees [°]")
title("Angular Position")

subplot(3,1,2)
plot(time, rad2deg(ThetaDotKalman))
grid on
ylabel("Degrees [°/s]")
title("Angular Velocity")

subplot(3,1,3)
plot(time, rad2deg(ThetaDDotKalman))
ylabel("Degrees [°/s^2]")
title("Angular Acceleration")
grid on