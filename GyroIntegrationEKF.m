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
 
AngleSim = sim("RateGyro_to_EulerAngles.slx");

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
ThetaX = phi(1); ThetaY = theta(1); ThetaZ = psi(1); 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = phi_dot(1); OmegaY = theta_dot(1); OmegaZ = psi_dot(1); 

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

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

AngleXKalman = [];
AngleYKalman = [];
AngleZKalman = [];
OmegaXKalman = [];
OmegaYKalman = [];
OmegaZKalman = [];

%% Need the standard deviation and residuals to evaluate the filter mathematically
PosThetaXSTD = [];
PosThetaYSTD = [];

SpeedThetaXSTD = [];
SpeedThetaYSTD = [];

ResidualThetaX = [];
ResidualThetaY = [];

ResidualOmegaX = [];
ResidualOmegaY = [];

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
    
    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | -cos(θx)cos(θy)| 
    
    % Using the positive version for az since the outout should be +1g
    
    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | cos(θx)cos(θy) | 
    
    % For gyro measurment model, I have already converted the (p,q,r) to
    % angular rates using simulink
    
    h_of_x = [cosd(Xkp(1))*sind(Xkp(2));
              sind(Xkp(1));
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
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
    AngleZKalman = [AngleZKalman; Xk(3)];
    OmegaXKalman = [OmegaXKalman; Xk(4)];
    OmegaYKalman = [OmegaYKalman; Xk(5)];
    OmegaZKalman = [OmegaZKalman; Xk(6)];
    
    PosThetaXSTD = [PosThetaXSTD; sqrt(Pk(1,1))];
    PosThetaYSTD = [PosThetaYSTD; sqrt(Pk(2,2))];
    SpeedThetaXSTD = [SpeedThetaXSTD; sqrt(Pk(4,4))];
    SpeedThetaYSTD = [SpeedThetaYSTD; sqrt(Pk(5,5))];

    ResidualThetaX = [ResidualThetaX; yk(1)];
    ResidualThetaY = [ResidualThetaY; yk(2)];
    ResidualOmegaX = [ResidualOmegaX; yk(4)];
    ResidualOmegaY = [ResidualOmegaY; yk(5)];
end 


% Plotting

figure(1)
plot(time, phi_dot)
title("Gyroscope \omegaX");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaXKalman)
legend("Measured Gyro data \omegaX", "Extended Kalman Filter Gyro data \omegaX")
hold off

figure(2)
plot(time, theta_dot)
title("Gyroscope \omegaY");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaYKalman)
legend("Measured Gyro data \omegaY", "Extended Kalman Filter Gyro\omegaY")
hold off

figure(3)
plot(time, psi_dot)
title("Gyroscope \omegaZ");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaZKalman)
legend("Measured Gyro data \omgeaZ", "Extended Kalman Filter Gyro\omegaZ")
hold off


figure(4)
plot(time, AngleXKalman)
title("Angle \thetaX (Roll)");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, phi)
legend("Extended Kalman Filter \thetaX", "Gyroscope \thetaX")
% legend("Extended Kalman Filter \thetaX")
hold off

figure(5)
plot(time, AngleYKalman)
title("Angle \thetaY (Pitch)");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, theta)
legend("Extended Kalman Filter \thetaY", "Gyroscope \thetaY")
%legend("Extended Kalman Filter \thetaY")
hold off

% Plotting Residuals
figure(6)
plot(time, 3*PosThetaXSTD, 'ko')
title("\thetaX Residuals 3\sigma")
ylabel("Degrees")
grid on
hold on
plot(time, ResidualThetaX)
plot(time, -3*PosThetaXSTD, 'ko')
hold off

figure(7)
plot(time, 3*PosThetaYSTD, 'ko')
title("\thetaY Residuals 3\sigma")
ylabel("Degrees")
grid on
hold on
plot(time, ResidualThetaY)
plot(time, -3*PosThetaYSTD, 'ko')
hold off

figure(8)
plot(time, 3*SpeedThetaXSTD, 'ko')
title("\omegaX Residuals 3\sigma")
ylabel("Degrees/sec")
grid on
hold on
plot(time, ResidualOmegaX)
plot(time, -3*SpeedThetaXSTD, 'ko')
hold off

figure(9)
plot(time, 3*SpeedThetaYSTD, 'ko')
title("\omegaY Residuals 3\sigma")
ylabel("Degrees/sec")
grid on
hold on
plot(time, ResidualOmegaY)
plot(time, -3*SpeedThetaYSTD, 'ko')
hold off

