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

MeasuredData = readtable("SampleData.xlsx"); 

addpath("UnscentedFilter")

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

%% Initializing the states and covariance
P = 500*eye(6);

states = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ];

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Qk = eye(length(P));

Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);

Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);

Wk = 0; 

Rk = eye(size(P))*0.3;

%% Values we want to plot 

AngleXKalman = [];
AngleYKalman = [];
AngleZKalman = [];
OmegaXKalman = [];
OmegaYKalman = [];
OmegaZKalman = [];

AngleXAccelerometer = [];
AngleYAccelerometer = [];

%% Need the standard deviation and residuals to evaluate the filter mathematically
PosThetaXSTD = [];
PosThetaYSTD = [];

SpeedThetaXSTD = [];
SpeedThetaYSTD = [];

ResidualThetaX = [];
ResidualThetaY = [];

ResidualOmegaX = [];
ResidualOmegaY = [];

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

    propagatedAccel = zeros(3,length(newSigmaPoints));

    %% Passing sigma points through non linear measurement model:
    % the measurement function converts the filter’s prior into a measurement

    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | -cos(θx)cos(θy)|
    
    % Using the positive version for az since the outout should be +1g
    
    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | cos(θx)cos(θy) | 

    for i = 1:length(propagatedAccel)
        propagatedAccel(1,i) = cosd(newSigmaPoints(1,i))*sind(newSigmaPoints(2,i));
        propagatedAccel(2,i) = sind(newSigmaPoints(2,i)); 
        propagatedAccel(3,i) = cosd(newSigmaPoints(1,i))*cosd(newSigmaPoints(2,i));
    end 

    % For gyro measurment model, I have already converted the (p,q,r) to
    % angular rates using simulink

    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));

    newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
    newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:end,:);  % gyro stays the same

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    % (1:3) = accelerometer, (4:6) = gyroscope
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii); phi_dot(iii); theta_dot(iii); psi_dot(iii)];
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
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
    AngleZKalman = [AngleZKalman; Xk(3)];
    OmegaXKalman = [OmegaXKalman; Xk(4)];
    OmegaYKalman = [OmegaYKalman; Xk(5)];
    OmegaZKalman = [OmegaZKalman; Xk(6)];
    
    AngleXAccelerometer = [AngleXAccelerometer; z(1)];
    AngleYAccelerometer = [AngleYAccelerometer; z(2)];
    
    PosThetaXSTD = [PosThetaXSTD; sqrt(Pk(1,1))];
    PosThetaYSTD = [PosThetaYSTD; sqrt(Pk(2,2))];
    SpeedThetaXSTD = [SpeedThetaXSTD; sqrt(Pk(4,4))];
    SpeedThetaYSTD = [SpeedThetaYSTD; sqrt(Pk(5,5))];

    ResidualThetaX = [ResidualThetaX; y(1)];
    ResidualThetaY = [ResidualThetaY; y(2)];
    ResidualOmegaX = [ResidualOmegaX; y(4)];
    ResidualOmegaY = [ResidualOmegaY; y(5)];
end 


%% Plotting

figure(1)
plot(time, phi_dot)
title("Gyroscope \omegaX");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaXKalman)
legend("Measured Gyro data \omegaX", "Unscented Kalman Filter Gyro data \omegaX")
hold off

figure(2)
plot(time, theta_dot)
title("Gyroscope \omegaY");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaYKalman)
legend("Measured Gyro data \omegaY", "Unscented Kalman Filter Gyro\omegaY")
hold off

figure(3)
plot(time, psi_dot)
title("Gyroscope \omegaZ");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaZKalman)
legend("Measured Gyro data \omgeaZ", "Unscented Kalman Filter Gyro\omegaZ")
hold off

figure(4)
plot(time, AngleXKalman)
title("Angle \thetaX (Roll)");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, phi)
legend("Unscented Kalman Filter \thetaX", "Gyroscope \thetaX")
% legend("Unscented Kalman Filter \thetaX")
hold off

figure(5)
plot(time, AngleYKalman)
title("Angle \thetaY (Pitch)");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, theta)
legend("Unscented Kalman Filter \thetaY", "Gyroscope \thetaY")
%legend("Unscented Kalman Filter \thetaY")
hold off

%% Plotting Residuals
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


