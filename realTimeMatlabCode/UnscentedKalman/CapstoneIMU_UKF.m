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
fileName = '/validation.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/UnscentedKalman';

addpath('UnscentedFilter');

%% Sensor info
port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

imu = mpu6050(a,'SampleRate', 100);

dt = 1/(imu.SampleRate);

startSample = 1;
stopSample = 800;

accel = zeros(stopSample, 3);   % [m/s^2]
gyro = zeros(stopSample, 3);    % [rad/s]

[OSX,OSY,OSZ] = calibrateGyro(imu);

fprintf("\n")

[xOff,yOff,zOff] = calibrateAccel(imu);

fprintf("\n")

fprintf("\nNow Gathering Data\n") 

states = [1;0;0;0]; % initial quaternion

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

Rk = eye(3)*0.045;

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
    
    % Normalizing 
     
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

    NewPrediction = stateProp(samplePoints, dt, Gyro, Wk);
    
    %% Compute the weights 
    [Wc, Wm] = weights(NewPrediction,alpha,beta);

    %% Perform the Unscented Transform by summing the sample mean and covariances
    %% With their respective weights to produce a new mean and covariance

    mean_vec = quatnormalize((NewPrediction*Wm).'); % Prior
    
    Mu_x = mean_vec.';

    Px = PredictCovarianceUKF(NewPrediction, samplePoints, Mu_x ,Wc, Qk);
    
    [~,EigenVal] = eig(Px);
    
    if EigenVal(EigenVal < 0) 
        [Px_new,~] = isSemiDef(Px);
    else 
        Px_new = Px;
    end 
        
    %% Measurements
    % First get the new sigma points from the newly calculated mean and
    % covariance

    newSigmaPoints = sigmaPoints(Mu_x,Px_new,alpha);

    %% Passing sigma points through non linear measurement model:
    % the measurement function converts the filter’s prior into a measurement
    
    AccelModel = MeasurementModel(newSigmaPoints);
    
    newMeasurementSigmaPoints = AccelModel;
    
    tempMu_z = newMeasurementSigmaPoints*Wm;
    
    accelMuNorm = tempMu_z(1:3)/norm(tempMu_z(1:3)); % Normalizing
    
    Mu_z = accelMuNorm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    
    z = [AccelX;AccelY;AccelZ];
    
    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*pinv(Pz);

    %% Compute the posterior using the prior and measurement residual
    y = z-Mu_z;
    
    %% Update the state
    Xk = Mu_x + K*y;
    Xk = quatnormalize(Xk.'); % [1x4]
    
    % Converting to Euler Angles
    [yaw, pitch, roll] = quat2angle(Xk);

    %% Update the covariance
    Pk = Px_new - K*(Pz)*K.';
    
    % Store for plotting
    PhiKalman = [PhiKalman;roll];
    ThetaKalman = [ThetaKalman;pitch];
    PsiKalman = [PsiKalman;yaw];
    
    %% Repeat for next iteration
    states = Xk.'; % [4x1]
    
    [~,EigenVal2] = eig(Pk);
    
    if EigenVal2(EigenVal2 < 0) 
        [Pk_new,~] = isSemiDef(Pk);
    else 
        Pk_new = Pk;
    end 
    
    P = Pk_new;
    
    %% Plotting
    subplot(3,1,1);
    grid on
    plot(rad2deg(PhiKalman))
    title("X-Axis Rotation")
    
    subplot(3,1,2);
    plot(rad2deg(ThetaKalman))
    title("Y-Axis Rotation")
    
    subplot(3,1,3);
    plot(rad2deg(PsiKalman))
    title("Z-Axis Rotation")

end 

fprintf("Storing Data\n")

T = table(PhiKalman,ThetaKalman,PsiKalman);

%% Creating excel sheet 
ExportSheet(fileName, filePath, T);

fprintf("\nData Exported\n")

function [OSX,OSY,OSZ] = calibrateGyro(imu)
    
    fprintf("Please do not move sensor while calibrating gyro\n")
    
    buffer = zeros(200, 3);
    
   for j = 1:length(buffer)*2 % Throwing out first 400 readings
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
    
   for j = 1:length(buffer)*2 % Throwing out first 400 readings
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

function qk = stateProp(sigmaPoints, dt, omega, wk)

possibleStates = zeros(size(sigmaPoints));

wx = omega(1);
wy = omega(2);
wz = omega(3);

qdot = (0.5 * dt) * [0 -wx -wy -wz;
                     wx  0  wz -wy;
                     wy -wz  0  wx;
                     wz  wy -wx  0];          
                 
I = eye(length(qdot));

F = (I + qdot); 

j = size(sigmaPoints);

% Propagating each column in my array of sigma points

    for i = 1:j(2)
        possibleStates(:,i) = F*sigmaPoints(:,i) + wk;
    end 

flipArray = quatnormalize(possibleStates.');

possibleStates = flipArray.'; 
    
% Adding noise

qk = possibleStates;

end 

function model = MeasurementModel(sigmaPoints)

newMeasurementSigmaPoints = zeros(3,length(sigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        
        qs = sigmaPoints(1,i);
        qx = sigmaPoints(2,i);
        qy = sigmaPoints(3,i);
        qz = sigmaPoints(4,i);
        
        newMeasurementSigmaPoints(1,i) = -2*(qx*qz - qy*qs);
        newMeasurementSigmaPoints(2,i) = -2*(qy*qz + qx*qs);
        newMeasurementSigmaPoints(3,i) = -(qs^2 -qx^2 - qy^2 + qz^2);
    end 

    model = newMeasurementSigmaPoints;
end 

function [P2,iter] = isSemiDef(P1)

[EigenVec,EigenVal] = eig(P1);

iter = 0;


    while EigenVal(EigenVal < 0) 
        
        % if there are negative eigenvalues, flip the sign
        EigenVal(EigenVal < 0) = EigenVal(EigenVal < 0) * -1; 

        % recalculating the covariance matrix with new eigenvalues
        P2 = EigenVec*EigenVal*EigenVec.';
        
        % checking the condition
        [EigenVec,EigenVal] = eig(P2);

        iter = iter + 1;
        
        % if there aren't anymore, finish
        if EigenVal(EigenVal > 0) 
            break
        end 
        
    end 


end 

function ExportSheet(fileName, filePath, table)

fileToSave = strcat(filePath, fileName);

writetable(table, fileToSave);

end 
