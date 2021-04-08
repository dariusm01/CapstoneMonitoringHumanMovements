
% arduinosetup();

%% Name the file to save
fileName = '/Trial1.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/ExtendedKalman/EKF_9DOF_Files';

%% Sensor info

port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

% Port: '/dev/cu.usbmodem401'
% Board: 'Mega2560'

imu = mpu9250(a,'SamplesPerRead', 100);

% SampleRate = 100 (samples/s)
dt = 1/100;

startSample = 1;
stopSample = 1500;

accel = zeros(stopSample, 3);   % [m/s^2]
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

% % Expected Field Strength 
% ExpField = @(x) sqrt((x.^2)/3);
% 
% % Magnitude at the Earth's surface ranges from 25 to 65 microteslas
% Field = ExpField(45);

fprintf("\nNow Gathering Data\n")

%% Prediction 

% x = Fx + Gu + w

F = [1 0 0; 
     0 1 0; 
     0 0 1];
 
G = [dt 0 0; 
     0 dt 0; 
     0 0 dt];
 
zk = zeros(6,1);

h_of_x = zk; 

% State Matrix
Xk_1 = [Phi; Theta; Psi];

% Covariance Matrix 
Pk_1 = eye(length(F))*500;

% Noise
AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Wk = 0;

Qk = eye(size(Pk_1))*GyroSpectralDensity;

% Measurement noise
Rk = eye(length(zk))*0.8;

H = ones(length(zk),length(Xk_1));

I = eye(size(Pk_1));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

for i = startSample:stopSample
    
    [accelReadings,~] = readAcceleration(imu);
    accel(i,:) = accelReadings / 9.81; % in G's
    
    [gyroReadings,~] = readAngularVelocity(imu);
    gyro(i,:) = gyroReadings - [OSX,OSY,OSZ];
    
    [magReadings,~] = readMagneticField(imu);
    mag(i,:) = magReadings;
    
    Mx(i) = (mag(i,1) - Offset(1))*Scale(1); % Hard Iron Correction & % Soft Iron Correction
    MagX = Mx(i); 
    
    My(i) = (mag(i,2) - Offset(2))*Scale(2);
    MagY = My(i);
    
    Mz(i) = (mag(i,3) - Offset(3))*Scale(3);
    MagZ = Mz(i);
    
    fieldMagnitude  = norm([MagX MagY MagZ]);
    
    MagX = MagX/fieldMagnitude;
    
    MagY = MagY/fieldMagnitude;
    
    MagZ = MagZ/fieldMagnitude;
    
    %% To NED Frame
    GyroX = gyro(i,2);
    GyroY = gyro(i,1);
    GyroZ = -gyro(i,3);
    
    Gyro = [GyroX;GyroY;GyroZ];
    
    AccelX = accel(i,2);
    AccelY = accel(i,1);
    AccelZ = -accel(i,3);
    
    accelMagnitude  = norm([AccelX AccelY AccelZ]);
    
    AccelX = AccelX/accelMagnitude;
    AccelY = AccelY/accelMagnitude;
    AccelZ = AccelZ/accelMagnitude;
    
    % Mag is already in NED
    
    %% Converting Gyro to Euler rates
    if Theta == 1.5708  % Lazy way of avoiding gimbal lock
        Theta = 1.5516;
    end 
    
    [phiDot,thetaDot,psiDot] = EulerRate(Phi,Theta, Gyro);
    
    % Euler Rates are inputs into the system
    u =  [phiDot;thetaDot;psiDot]; 
    
    %% Extended Kalman Filter
    
    % Prior
    Xkp = F*Xk_1 + G*u + Wk;  

    % Proccess Covariance matrix
    Mk = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Mk*H.' + Rk;
    
    % Measurement (evidence)
    zk = [AccelX; AccelY; AccelZ; MagX; MagY; MagZ];
    
    % Measurement Model Accelerometer
    [ax,ay,az] = AccelModel(Xkp(1), Xkp(2));
    
    % Measurement Model Magnetometer
    [mx,my,mz] = MagnetModel(Xkp(2), Xkp(3));
    
    h_of_x = [ax;ay;az;mx;my;mz];
    
    H = MeasurementJacobian(Xkp(1),Xkp(2), Xkp(3));
    
    % Innovation (Residual)
    yk = zk - h_of_x;
    
    % Kalman Gain  
    K = Mk*H.'*pinv(Sk);
    
    % Posterior 
    Xk = Xkp + K*yk;
    
    Phi = Xk(1);
    Theta = Xk(2);
    Psi = Xk(3);
    
    % Covariance Update
    Pk = (I - K*H)*Mk*(I - K*H).' + (K*Rk*K.');
    
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

    % Redefining for next iteration
    Xk_1 = Xk;
    
    Pk_1 = Pk;
end 
    
fprintf("Storing Data\n")

T = table(PhiKalman,ThetaKalman,PsiKalman);

%% Creating excel sheet 
ExportSheet(fileName, filePath, T);

fprintf("\nData Exported\n")


function [OSX,OSY,OSZ] = calibrateGyro(imu)

    fprintf("Calibrating Gyroscope :\n")
    
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
    
function ExportSheet(fileName, filePath, table)

fileToSave = strcat(filePath, fileName);

writetable(table, fileToSave);

end 

function [phiDot,thetaDot,psiDot] = EulerRate(phi,theta, Gyro)


EulerKinematic = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 
                  0 cos(phi) -sin(phi);
                  0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
              
EulerRates = EulerKinematic*Gyro;

phiDot = EulerRates(1);
thetaDot = EulerRates(2);
psiDot = EulerRates(3);

end 

function H = MeasurementJacobian(phi,theta,psi)

H = [0 -cos(theta) 0; 
    cos(theta)*cos(phi) -sin(theta)*sin(phi) 0; 
    -cos(theta)*sin(phi) -sin(theta)*cos(phi) 0;
    0 -sin(theta)*cos(psi) -cos(theta)*sin(psi);
    0 -sin(theta)*sin(psi)  cos(theta)*cos(psi);
    0  -cos(theta)          0];
   
end 


function [mx,my,mz] = MagnetModel(theta,psi)

mx = cos(theta)*cos(psi);

my = cos(theta)*sin(psi);

mz = -sin(theta);

end 

function [ax,ay,az] = AccelModel(Phi, Theta)

ax = - sin(Theta);

ay = cos(Theta)*sin(Phi);

az = cos(Theta)*cos(Phi);
end 
