
% arduinosetup();

%% Name the file to save
fileName = '/QuaternionDemo_FinalPresentation.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/UnscentedKalman/UKF_9DOF';

addpath('UnscentedFilter');

%% Sensor info
port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

imu = mpu9250(a,'SampleRate', 100);

dt = 1/(imu.SampleRate);

startSample = 1;
stopSample = 1000;

accel = zeros(stopSample, 3);   % [m/s^2]
gyro = zeros(stopSample, 3);    % [rad/s]
mag = zeros(stopSample, 3);     % [µT]

Mx = zeros(stopSample, 1);
My = zeros(stopSample, 1);
Mz = zeros(stopSample, 1);

[OSX,OSY,OSZ] = calibrateGyro(imu);

fprintf("\n")

[xOff,yOff,zOff] = calibrateAccel(imu);

fprintf("\n")

[Offset, Scale] = CalibrateMag(imu);
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

Rk = eye(6)*0.045;

Rk(4,4) = 0.1;

Rk(5,5) = 0.1;

Rk(6,6) = 0.1;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

% Magnetic declination angle

% https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

magDip = dms2degrees([-11 8 25]); % -11° 8' 25" using my coordinates

magDip = deg2rad(magDip);

for iii = startSample:stopSample
    
    [accelReadings,~] = readAcceleration(imu);
    accel(iii,:) = (accelReadings / 9.81) - [xOff,yOff,zOff]; % in G's
    
    [gyroReadings,~] = readAngularVelocity(imu);
    gyro(iii,:) = gyroReadings - [OSX,OSY,OSZ];
    
    [magReadings,~] = readMagneticField(imu);
    mag(iii,:) = magReadings;
    
    Mx(iii) = (mag(iii,1) - Offset(1))*Scale(1); % Hard Iron Correction & % Soft Iron Correction
    MagX = Mx(iii); 
    
    My(iii) = (mag(iii,2) - Offset(2))*Scale(2);
    MagY = My(iii);
    
    Mz(iii) = (mag(iii,3) - Offset(3))*Scale(3);
    MagZ = Mz(iii);

    fieldMagnitude  = norm([MagX MagY MagZ]);
    
    % Normalizing (we care more about the direction than magnitude)
    
    MagX = MagX/fieldMagnitude;
    
    MagY = MagY/fieldMagnitude;
    
    MagZ = MagZ/fieldMagnitude;
    
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
    
    AccelMagModel = MeasurementModel(newSigmaPoints,magDip);
    
    newMeasurementSigmaPoints = AccelMagModel;
    
    tempMu_z = newMeasurementSigmaPoints*Wm;
    
    accelMuNorm = tempMu_z(1:3)/norm(tempMu_z(1:3)); % Normalizing
    
    magMuNorm = tempMu_z(4:6)/norm(tempMu_z(4:6)); % Normalizing
    
    Mu_z = [accelMuNorm;magMuNorm];

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    
    z = [AccelX;AccelY;AccelZ;MagX;MagY;MagZ];
    
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

function [Offsets, Scale] = CalibrateMag(imu)

    fprintf("Calibrating Magnetometer:\n")

    fprintf("Please move the sensor in a figure 8 pattern to collect samples at different orientations\n")

    buffer = zeros(200, 3);
    
    for j = 1:length(buffer)*2 % Throwing out first 400 readings
       [~,~] = readMagneticField(imu);
    end 

    for i = 1:length(buffer)
       [magSamples,~] = readMagneticField(imu);
       buffer(i,:) = magSamples; 
    end 
    
    MagX =  buffer(:,1);
    MagY =  buffer(:,2);
    MagZ =  buffer(:,3);
    
    %% Hard Iron Correction

    MagXOffset = (max(MagX)+min(MagX))/2;
    MagYOffset = (max(MagY)+min(MagY))/2;
    MagZOffset = (max(MagZ)+min(MagZ))/2;

    MagXHI = MagX-MagXOffset;
    MagYHI = MagY-MagYOffset;
    MagZHI = MagZ-MagZOffset;

    %% Soft Iron Correction 
    chordX = (max(MagXHI) - min(MagXHI))/2;
    chordY = (max(MagYHI) - min(MagYHI))/2;
    chordZ = (max(MagZHI) - min(MagZHI))/2;

    chord_average = (chordX + chordY + chordZ)/3;

    MagXScale = chord_average/chordX;
    MagYScale = chord_average/chordY;
    MagZScale = chord_average/chordZ;
    
    Offsets = [MagXOffset MagYOffset MagZOffset];
    Scale = [MagXScale MagYScale MagZScale];
    
    fprintf("Magnetometer Calibration Complete\n")
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

function model = MeasurementModel(sigmaPoints,beta)

newMeasurementSigmaPoints = zeros(6,length(sigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        
        qs = sigmaPoints(1,i);
        qx = sigmaPoints(2,i);
        qy = sigmaPoints(3,i);
        qz = sigmaPoints(4,i);
        
        newMeasurementSigmaPoints(1,i) = -2*(qx*qz - qy*qs);
        newMeasurementSigmaPoints(2,i) = -2*(qy*qz + qx*qs);
        newMeasurementSigmaPoints(3,i) = -(qs^2 -qx^2 - qy^2 + qz^2);
        newMeasurementSigmaPoints(4,i) = (qs^2 + qx^2 - qy^2 - qz^2)*cos(beta) + 2*(qx*qz - qy*qs)*sin(beta);
        newMeasurementSigmaPoints(5,i) = 2*(qx*qy - qz*qs)*cos(beta) + 2*(qy*qz + qx*qs)*sin(beta);
        newMeasurementSigmaPoints(6,i) = 2*(qx*qz + qy*qs)*cos(beta) + (qs^2 -qx^2 - qy^2 + qz^2)*sin(beta);
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
