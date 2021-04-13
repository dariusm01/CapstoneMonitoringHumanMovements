
% arduinosetup();

%% Name the file to save
fileName = '/Trial1_no_accel.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/UnscentedKalman';

addpath('UnscentedFilter');

%% Sensor info
port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

imu = mpu9250(a,'SamplesPerRead', 100);

% SampleRate = 100 (samples/s)
dt = 1/100;

startSample = 1;
stopSample = 1500;

gyro = zeros(stopSample, 3);    % [rad/s]
mag = zeros(stopSample, 3);     % [µT]

Mx = zeros(stopSample, 1);
My = zeros(stopSample, 1);
Mz = zeros(stopSample, 1);

[Offset, Scale] = CalibrateMag(imu);
fprintf("\n")

[OSX,OSY,OSZ] = calibrateGyro(imu);

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

Rk = eye(size(P))*0.085;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

for iii = startSample:stopSample
    
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
    
    totalMag = [Mx My Mz];
    
    fieldMagnitude  = norm([MagX MagY MagZ]);
    
    MagX = MagX/fieldMagnitude;
    
    MagY = MagY/fieldMagnitude;
    
    MagZ = MagZ/fieldMagnitude;
    
    %% To NED Frame
    GyroX = gyro(iii,2);
    GyroY = gyro(iii,1);
    GyroZ = -gyro(iii,3);
    
    Gyro = [GyroX;GyroY;GyroZ];
    
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
    
    % Measurement Model Magnetometer ψ
    
    % |mx|     |  cos(θ)cos(ψ) |
    % |my|  =  |  cos(θ)sin(ψ) |
    % |mz|     |    -sin(ψ)    |

 
    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        newMeasurementSigmaPoints(1,i) = cos(newSigmaPoints(2,i))*cos(newSigmaPoints(3,i));
        newMeasurementSigmaPoints(2,i) = cos(newSigmaPoints(2,i))*sin(newSigmaPoints(3,i)); 
        newMeasurementSigmaPoints(3,i) = -sin(newSigmaPoints(3,i));
    end 


    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    z = [MagX; MagY; MagZ];
    
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
    PsiKalman = [PsiKalman; Xk(3)];
    
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

function ExportSheet(fileName, filePath, table)

fileToSave = strcat(filePath, fileName);

writetable(table, fileToSave);

end 

function [Offsets, Scale] = CalibrateMag(imu)

    fprintf("Calibrating Magnetometer :\n")

    fprintf("Please move the sensor in a figure 8 pattern to collect samples at different orientations\n")

    buffer = zeros(200, 3);
    
    for j = 1:length(buffer)*7 % Throwing out first 1400 readings
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

