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

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

%% Initializing the states and covariance
P = 500*eye(6);

states = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ];

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

%% First, gather sigma points

% The code accepts the states as a column vector (states x 1) like normal
% It then outputs a (states x sigma points) matrix
samplePoints = sigmaPoints(states,P,alpha); 

%% Then, pass the sigma points through your model (Prediction)
dt = 1/500;

% Input the epoch (dt), sigma Points, and noise (wk)
NewPrediction = firstOrderUKFPropagation(dt, samplePoints, 0);

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

% Using the positive version for az since the outout should be +1g

% |ax|     | cos(θx)sin(θy) |
% |ay|  =  |     sin(θx)    |
% |az|     | cos(θx)cos(θy) | 

for i = 1:length(propagatedAccel)
    propagatedAccel(1,i) = cosd(newSigmaPoints(1,i))*sind(newSigmaPoints(2,i));
    propagatedAccel(2,i) = sind(newSigmaPoints(2,i)); 
    propagatedAccel(3,i) = cosd(newSigmaPoints(1,i))*cosd(newSigmaPoints(2,i));
end 

% For gyro measurment model, it would be best to use
% ω_true = ω_output - bias
% bias is not calclated, so I am skipping it for now

newMeasurementSigmaPoints = zeros(size(newSigmaPoints));

newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:end,:);  % gyro stays the same

Mu_z = newMeasurementSigmaPoints*Wm;

%% Measurment covariance

% Measurement noise
Rk = eye(size(P))*0.1;
Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

% measurements from sensor
% (1:3) = accelerometer, (4:6) = gyroscope
sensorReadings = [-0.0008; 0.0002; 0.9955; 0.2253; -0.1456; 0.4119];

z = sensorReadings;

%% Cross Covariance
Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

%% Kalman Gain
K = Pxz*(Pz)^-1;

%% Compute the posterior using the prior and measurement residual
y = z-Mu_z;

Xk = Mu_x + K*y;

%% Update the covariance
Pk = Px - K*(Pz)*K.';