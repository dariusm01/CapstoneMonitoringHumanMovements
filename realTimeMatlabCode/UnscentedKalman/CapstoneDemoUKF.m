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
%  11. Calculate the cross covariance (Pxz)
%  12. Determine the kalman gain (K)
%  13. Use the kalman gain to find the estimation (posterior)
%  14. Update the covariance 
%  15. Repeat for next time step

% arduinosetup();

%% Name the file to save
fileName = '/newTrial3.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/UnscentedKalman';

addpath('UnscentedFilter');

%% Sensor info
port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

% Port: '/dev/cu.usbmodem401'
% Board: 'Mega2560'

imu = mpu6050(a,'SampleRate', 100);

dt = 1/(imu.SampleRate);

startSample = 1;
stopSample = 3000;

accel = zeros(stopSample, 3);   % [m/s^2]
gyro = zeros(stopSample, 3);    % [rad/s]

[OSX,OSY,OSZ] = calibrateGyro(imu);

fprintf("\n")

[xOff,yOff,zOff] = calibrateAccel(imu);

% Initial conditions
Phi = 0; 
Theta = 0;
Psi = 0;

fprintf("\nNow Gathering Data\n") 

%% Initializing the states and covariance

states = [Phi; Theta; Psi];

P = 500*eye(length(states));

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

% Noise
AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Wk = 0;

Qk = eye(size(P))*GyroSpectralDensity;

Rk = eye(size(P))*0.22;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

for iii = startSample:stopSample
    
    [accelReadings,~] = readAcceleration(imu);
    accel(iii,:) = (accelReadings / 9.81) - [xOff,yOff,zOff]; % in G's
    
    [gyroReadings,~] = readAngularVelocity(imu);
    gyro(iii,:) = gyroReadings - [OSX,OSY,OSZ];
    
    %% To NED Frame
    GyroX = gyro(iii,2);
    GyroY = gyro(iii,1);
    GyroZ = -gyro(iii,3);
    
    Gyro = [GyroX;GyroY;GyroZ];
    
    AccelX = accel(iii,2);
    AccelY = accel(iii,1);
    AccelZ = -accel(iii,3);
    
    accelMag = norm([AccelX AccelY AccelZ]);
    
    AccelX = AccelX/accelMag;
    AccelY = AccelY/accelMag;
    AccelZ = AccelZ/accelMag;
    
    %% First, gather sigma points

    % The code accepts the states as a column vector (states x 1) like normal
    % It then outputs a (states x sigma points) matrix
    samplePoints = sigmaPoints(states,P,alpha); 

    %% Then, pass the sigma points through your model (Prediction)
    % Input the epoch (dt), sigma Points, and noise (wk)
    NewPrediction = firstOrderUKFPropagation(samplePoints, dt, Gyro, Wk);
    
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
    
    % |ax|     |     sin(θ)    |
    % |ay|  =  | -cos(θ)sin(φ) |
    % |az|     | -cos(θ)cos(φ) |
 
    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        newMeasurementSigmaPoints(1,i) = sin(newSigmaPoints(2,i));
        newMeasurementSigmaPoints(2,i) = - cos(newSigmaPoints(2,i))*sin(newSigmaPoints(1,i)); 
        newMeasurementSigmaPoints(3,i) = - cos(newSigmaPoints(2,i))*cos(newSigmaPoints(1,i));
    end 

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    z = [AccelX; AccelY; AccelZ];
    
    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*pinv(Pz);

    %% Compute the posterior using the prior and measurement residual
    y = z-Mu_z;
    
    %% Update the state
    Xk = Mu_x + K*y;

    %% Update the covariance
    Pk = Px - K*(Pz)*K.';
    
    % Store for plotting
    PhiKalman = [PhiKalman; Xk(1)];
    ThetaKalman = [ThetaKalman; Xk(2)];
    PsiKalman = [PsiKalman; Xk(3)]; % drift
    
    %% Plotting
    
    subplot(3,1,1);
    plot(rad2deg(PhiKalman))
    title("X-Axis Rotation")
    
    subplot(3,1,2);
    plot(rad2deg(ThetaKalman))
    title("Y-Axis Rotation")

    subplot(3,1,3);
    plot(rad2deg(PsiKalman))
    title("Z-Axis Rotation")
    
    %% Repeat for next iteration
    states = Xk;
    P = Pk;
end 

fprintf("Storing Data\n")

T = table(PhiKalman,ThetaKalman,PsiKalman);

%% Creating excel sheet 
ExportSheet(fileName, filePath, T);

fprintf("\nData Exported\n")

function [OSX,OSY,OSZ] = calibrateGyro(imu)
    
    fprintf("Please do not move sensor while calibrating gyro\n")
    
    buffer = zeros(200, 3);
    
   for j = 1:length(buffer)*5 % Throwing out first 1000 readings
       [~,~] = readAngularVelocity(imu);
   end 
   
   for i = 1:length(buffer)
       [gyroSamples,~] = readAngularVelocity(imu);
       buffer(i,:) = gyroSamples; 
   end 
   
   OSX = mean(buffer(:,1));
   
   OSY = mean(buffer(:,2));
   
   OSZ = mean(buffer(:,3));
   
   fprintf("Gyroscope Calibration Complete\n")
   
end 

function [xOff, yOff, zOff] = calibrateAccel(imu)

fprintf("Please do not move sensor while calibrating the accelerometer\n")
    
    buffer = zeros(200, 3);
    
   for j = 1:length(buffer)*5 % Throwing out first 1000 readings
       [~,~] = readAcceleration(imu);
   end 
   
   for i = 1:length(buffer)
       [accelSamples,~] = readAcceleration(imu);
       buffer(i,:) = accelSamples / 9.81; % in G's
   end 
   
   xOff = mean(buffer(:,1));
   
   yOff = mean(buffer(:,2));
   
   zOff = mean(buffer(:,3)) - 1;
   
   fprintf("Accelerometer Calibration Complete\n")

end 

function ExportSheet(fileName, filePath, table)

fileToSave = strcat(filePath, fileName);

writetable(table, fileToSave);

end 

