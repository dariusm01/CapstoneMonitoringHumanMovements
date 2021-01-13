load('imuInfo.mat')

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

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
 
% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 

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
Rk = eye(size(Pk_1))*0.3;

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
    
    % zk = [AccelX(i); AccelY(i); AccelZ(i); phi_dot(i); theta_dot(i); psi_dot(i)];  
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
    
    
    h_of_x = [-sin(Xkp(2));
              cos(Xkp(2))*sin(Xkp(1));
              cos(Xkp(2))*cos(Xkp(1));
              Xkp(4)-(Xkp(6)*sin(Xkp(2)));
              Xkp(5)*cos(Xkp(1)) + Xkp(6)*cos(Xkp(2))*sin(Xkp(1));
              Xkp(6)*cos(Xkp(2))*cos(Xkp(1)) - Xkp(5)*sin(Xkp(1))];
    
          
    %% Measurment Jacobian 
    % the measurement function (H) converts the filter’s prior into a measurement
    H(1,1) = 0; H(1,2) = -cos(Xkp(2)); 
    H(2,1) = cos(Xkp(1))*cos(Xkp(2)); H(2,2) = -sin(Xkp(1))*sin(Xkp(2));
    H(3,1) = -cos(Xkp(2))*sin(Xkp(1)); H(3,2) = -cos(Xkp(1))*sin(Xkp(2)); H(3,3) = 0;
    H(4,2) = -Xkp(6)*cos(Xkp(2)); H(4,6) = -sin(Xkp(2));
    H(5,1) = Xkp(6)*cos(Xkp(1))*cos(Xkp(2));
    H(5,2) = -Xkp(6)*sin(Xkp(1))*sin(Xkp(2));
    H(5,3) = -Xkp(5)*sin(Xkp(3));
    H(5,5) = cos(Xkp(3));
    H(5,6) = cos(Xkp(2))*sin(Xkp(1));
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
plot(time, rad2deg(phi_dot))
title('Roll Angle Rate $\dot{\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(PhiDotKalman))
legend("Measured Gyro data \phi", "Extended Kalman Filter Gyro data \phi")
hold off

figure(2)
plot(time, rad2deg(theta_dot))
title('Pitch Angle Rate $\dot{\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(ThetaDotKalman))
legend("Measured Gyro data \theta", "Extended Kalman Filter Gyro \theta")
hold off

figure(3)
plot(time, rad2deg(psi_dot))
title('Yaw Angle Rate $\dot{\psi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(PsiDotKalman))
legend("Measured Gyro data \psi", "Extended Kalman Filter Gyro \psi")
hold off


figure(4)
plot(time, rad2deg(PhiKalman))
title('Roll Angle ${\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(phi))
legend("Extended Kalman Filter \phi", "Gyroscope \phi")
% legend("Extended Kalman Filter \Phi")
hold off

figure(5)
plot(time, rad2deg(ThetaKalman))
title('Pitch Angle ${\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(theta))
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

