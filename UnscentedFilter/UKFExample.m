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

%% Initializing the states and covariance
P = eye(6);

states = [0;0;0;0.2253;-.1456;0.4119];

beta = 2;
kappa = 3-length(states);
alpha = 0.1;

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

% |ax|     | cos(θx)sin(θy) |
% |ay|  =  |     sin(θx)    |
% |az|     | -cos(θx)cos(θy)|

propagatedAccel = zeros(3,length(newSigmaPoints));

%% Passing sigma points through non linear measurement model

for i = 1:length(propagatedAccel)
    propagatedAccel(1,i) = cos(newSigmaPoints(1,i))*sin(newSigmaPoints(2,i));
    propagatedAccel(2,i) = sin(newSigmaPoints(2,i)); 
    propagatedAccel(3,i) = -cos(newSigmaPoints(1,i))*cos(newSigmaPoints(2,i));
end 

% For gyro measurment model, it would be best to use
% ω_true = ω_output - bias
% bias is not calclated, so I am skipping it for now

newMeasurementSigmaPoints = zeros(size(newSigmaPoints));

newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:end,:);  % gyro stays the same

Mu_z_accel = newMeasurementSigmaPoints*Wm;

%% Measurment covariance

Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z_accel ,Wc, 0);

% converting accelerations into angles
measurementAccels = [accelAngleX(Mu_z_accel); accelAngleY(Mu_z_accel); 0];

Mu_z = zeros(size(Mu_z_accel));

Mu_z(1:3) = measurementAccels;
Mu_z(4:end) = Mu_z_accel(4:end);

% measurements from sensor

% (1:3) = accelerometer, (4:6) = gyroscope
sensorReadings = [-0.0008; 0.0002; 0.9955; 0.2253; -0.1456; 0.4119];

% converting accelerations into angles
convertedReading = [accelAngleX(sensorReadings); accelAngleY(sensorReadings); 0];

z = zeros(size(Mu_z_accel));

z(1:3) = convertedReading;
z(4:end) = sensorReadings(4:end);

%% Cross Covariance
% Still need to convert newMeasurementSigmaPoints(1:3,:) to angles
% Units don't match for the measurement
% Just testing the function to make sure there are no errors
Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

%% Kalman Gain
K = Pxz*(Pz)^-1;

%% Compute the posterior using the prior and measurement residual
Xk = Mu_x + K*(z-Mu_z);

%% Update the covariance
Pk = Px - K*(Pz)*K.';

%% Repeat for next iteration
states = Xk;
P = Pk;
samplePoints = sigmaPoints(states,P,alpha);
NewPrediction = firstOrderUKFPropagation(dt, samplePoints, 0);
[Wc, Wm] = weights(NewPrediction,alpha,beta);
Mu_x = NewPrediction*Wm; % Prior
Px = PredictCovarianceUKF(NewPrediction, samplePoints, Mu_x ,Wc, 0);
newSigmaPoints = sigmaPoints(Mu_x,Px,alpha);