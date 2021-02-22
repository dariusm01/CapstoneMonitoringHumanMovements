% An example of the first iteration of the unscented kalman filter for 
% our project. 
%   The steps include:
%   1. Initializing the states and covariance
%   2. Calculate sigma points (cholesky decomp for matrix sqrt)
%   3. Prediction - > plug in sigma points into prediction equation
%   4. Calculate the weights Wm & Wc (mean and covariance)
%   5. Perform the Unscented Transform to get μx (prior) and Px
%   6. Measurements -> use the μx & Px to find new sigma points
%   7. Plug the new sigma points into the measurement model equation(s) 
%   8. Find μz (unscented transform)
%   9. Take measurement (z)
%  10. Find Pz (measurement covariance)
%  11. Calculate the cross - covariance (Pxz)
%  12. Determine the kalman gain (K)
%  13. Use the kalman gain to find the estimation (posterior)
%  14. Update the covariance 
%  15. Repeat for next time step

addpath("/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/UnscentedFilter");

load("MPU6050_newSamples.mat")

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

dt = 1/100;

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
PhiDot = phi_dot(1); ThetaDot = theta_dot(1); PsiDot = psi_dot(1); 

%% Initializing the states and covariance
P = 500*eye(6);

states = [phi(1); theta(1); psi(1); PhiDot; ThetaDot; PsiDot];

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

% Noise
AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Wk = 0;

Qk = FirstOrderPNC(GyroSpectralDensity,dt);

Rk = eye(size(P))*0.3;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];
PhiDotKalman = [];
ThetaDotKalman = [];
PsiDotKalman = [];

% Sensor was not moving so everything should be close to 0
True = 0;

%% Need the standard deviation and residuals to evaluate the filter mathematically
PosPhiSTD = [];
PosThetaSTD = [];

SpeedPhiSTD = [];
SpeedThetaSTD = [];

ResidualPhi = [];
ResidualTheta = [];

ResidualPhiDot = [];
ResidualThetaDot = [];

for iii = 1:length(time)
    %% First, gather sigma points

    % The code accepts the states as a column vector (states x 1) like normal
    % It then outputs a (states x sigma points) matrix
    samplePoints = sigmaPoints(states,P,alpha); 

    %% Then, pass the sigma points through your model (Prediction)
    % Input the epoch (dt), sigma Points, and noise (wk)
    NewPrediction = firstOrderUKFPropagation(dt, samplePoints, Wk);

    %% Compute the weights 
    [Wc, Wm] = weights(NewPrediction,alpha,beta);

    %% Perform the Unscented Transform by summing the sample mean and covariances
    %% With their respective weights to produce a new mean and covariance

    Mu_x = NewPrediction*Wm; % Prior

    Px = PredictCovarianceUKF(NewPrediction, samplePoints, Mu_x ,Wc, Qk);

    %% Measurements
    % First get the new sigma points from the newly calculated mean and
    % covariance

    newSigmaPoints = sigmaPoints(Mu_x,Px,alpha);

    %% Passing sigma points through non linear measurement model:
    % the measurement function converts the filter’s prior into a measurement
    
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

    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        newMeasurementSigmaPoints(1,i) = -sin(newSigmaPoints(2,i));
        newMeasurementSigmaPoints(2,i) = cos(newSigmaPoints(2,i))*sin(newSigmaPoints(1,i)); 
        newMeasurementSigmaPoints(3,i) = cos(newSigmaPoints(2,i))*cos(newSigmaPoints(1,i));
        newMeasurementSigmaPoints(4,i) = newSigmaPoints(4,i) - newSigmaPoints(6,i)*sin(newSigmaPoints(2,i));
        newMeasurementSigmaPoints(5,i) = newSigmaPoints(5,i)*cos(newSigmaPoints(1,i)) + newSigmaPoints(6,i)*cos(newSigmaPoints(2,i))*sin(newSigmaPoints(1,i));
        newMeasurementSigmaPoints(6,i) = newSigmaPoints(6,i)*cos(newSigmaPoints(2,i))*cos(newSigmaPoints(1,i)) - newSigmaPoints(5,i)*sin(newSigmaPoints(1,i));
    end 

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    % (1:3) = accelerometer, (4:6) = gyroscope
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii); GyroX(iii); GyroY(iii); GyroZ(iii)];
    z = sensorReadings;
    
    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*(Pz)^-1;

    %% Compute the posterior using the prior and measurement residual
    y = z-Mu_z;
    
    %% Update the state
    Xk = Mu_x + K*y;

    %% Update the covariance
    Pk = Px - K*(Pz)*K.';
    
    %% Repeat for next iteration
    states = Xk;
    P = Pk;
    
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

    ResidualPhi = [ResidualPhi; True-Xk(1)];
    ResidualTheta = [ResidualTheta; True-Xk(2)];
    ResidualPhiDot = [ResidualPhiDot; True-Xk(4)];
    ResidualThetaDot = [ResidualThetaDot; True-Xk(5)];
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
legend("Measured Gyro data \phi", "Unscented Kalman Filter Gyro data \phi")
hold off

figure(2)
plot(time, rad2deg(theta_dot),'LineWidth',1)
title('Pitch Angle Rate $\dot{\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(ThetaDotKalman),'LineWidth',1)
legend("Measured Gyro data \theta", "Unscented Kalman Filter Gyro \theta")
hold off

figure(3)
plot(time, rad2deg(psi_dot),'LineWidth',1)
title('Yaw Angle Rate $\dot{\psi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees Per Second [°/s]")
grid on
hold on 
plot(time, rad2deg(PsiDotKalman),'LineWidth',1)
legend("Measured Gyro data \psi", "Unscented Kalman Filter Gyro \psi")
hold off


figure(4)
plot(time, rad2deg(PhiKalman),'LineWidth',1)
title('Roll Angle ${\phi}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(phi),'LineWidth',1)
legend("Unscented Kalman Filter \phi", "Gyroscope \phi")
% legend("Unscented Kalman Filter \Phi")
hold off

figure(5)
plot(time, rad2deg(ThetaKalman),'LineWidth',1)
title('Pitch Angle ${\theta}$','interpreter','latex')
xlabel("Time(s)")
ylabel("Degrees [°]")
grid on
hold on 
plot(time, rad2deg(theta),'LineWidth',1)
legend("Unscented Kalman Filter \theta", "Gyroscope \theta")
%legend("Unscented Kalman Filter \Theta")
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

