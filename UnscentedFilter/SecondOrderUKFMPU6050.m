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

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

ThetaZMag = zeros(size(GyroZ)); %no magnetometer

% Acceleration is also hidden - hard to initalize
AlphaX = 0; AlphaY = 0; AlphaZ = 0;

%% Initializing the states and covariance
P = 500*eye(9);

states = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ; AlphaX; AlphaY; AlphaZ];

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.9;

% AccelSpectralDensity = 300e-6*sqrt(dt);
% 
% GyroSpectralDensity = 0.01*sqrt(dt);

% Qk = eye(length(P));
% 
% Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);
% 
% Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);
Qk = 0;
Wk = 0; 

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
    NewPrediction = secondOrderUKFPropagation(dt, samplePoints, 0);

    %% Compute the weights 
    [Wc, Wm] = weights(NewPrediction,alpha,beta);

    %% Perform the Unscented Transform by summing the sample mean and covariances
    %% With their respective weights to produce a new mean and covariance

    Mu_x = NewPrediction*Wm; % Prior

    Px = PredictCovarianceUKF(NewPrediction, samplePoints, Mu_x ,Wc, 0);

    %% Measurements
    % First get the new sigma points from the newly calculated mean and
    % covariance

    newSigmaPoints = sigmaPoints(Mu_x,Px,alpha);

    propagatedAccel = zeros(3,length(newSigmaPoints));

    %% Passing sigma points through non linear measurement model:

    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | -cos(θx)cos(θy)|

    for i = 1:length(propagatedAccel)
        propagatedAccel(1,i) = cos(newSigmaPoints(1,i))*sin(newSigmaPoints(2,i));
        propagatedAccel(2,i) = sin(newSigmaPoints(2,i)); 
        propagatedAccel(3,i) = -cos(newSigmaPoints(1,i))*cos(newSigmaPoints(2,i));
    end 

    % For gyro measurment model, it would be best to use
    % ω_true = ω_output - bias
    % bias is not calclated, so I am skipping it for now
    % cannot measure the angular acceleration so it is not included

    newMeasurementSigmaPoints = zeros(6,length(newSigmaPoints));

    newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
    newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:6,:);  % gyro stays the same

    %% Converting the measurement sigma points into angles for the cross cov.

    for j = 1:length(newMeasurementSigmaPoints)
        newMeasurementSigmaPoints(1,j) = atan2(newMeasurementSigmaPoints(2,j), sqrt((newMeasurementSigmaPoints(1,j))^2 +...
            (newMeasurementSigmaPoints(3,j))^2)) * (180/pi);

        newMeasurementSigmaPoints(2,j) = atan2(-newMeasurementSigmaPoints(1,j), sqrt((newMeasurementSigmaPoints(2,j))^2 +...
            (newMeasurementSigmaPoints(3,j))^2))* (180/pi);

        newMeasurementSigmaPoints(3,j) = 0; % assuming the heading is 0 (no mag)
    end 

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, 0);

    % measurements from sensor

    % (1:3) = accelerometer, (4:6) = gyroscope
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii); GyroX(iii); GyroY(iii); GyroZ(iii)];

    % converting accelerations into angles
    convertedReading = [accelAngleX(sensorReadings); accelAngleY(sensorReadings); 0];

    z = zeros(size(Mu_z));

    z(1:3) = convertedReading;
    z(4:end) = sensorReadings(4:end);

    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*(Pz)^-1;

    %% Compute the posterior using the prior and measurement residual
    y = z-Mu_z;

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