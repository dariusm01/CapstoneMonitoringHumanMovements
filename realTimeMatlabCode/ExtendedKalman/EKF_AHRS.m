
% arduinosetup();

%% Name the file to save
fileName = '/AHRS_Trial2.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode/ExtendedKalman/EKF_9DOF_Files';

%% Sensor info

port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

imu = mpu9250(a,'SamplesPerRead', 100);

dt = 1/100;

startSample = 1;
stopSample = 3000;

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

% Initial conditions
Phi = 0; 
Theta = 0;
Psi = 0;

fprintf("\nNow Gathering Data\n")

%% Prediction 

% x = Fx + Gu + w

F = [1 0 0; 
     0 1 0; 
     0 0 1];
 
G = [dt 0 0; 
     0 dt 0; 
     0 0 dt];
 
zk = [0;0;0];

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
Rk = eye(size(Pk_1))*0.085;

H = eye(size(Pk_1));

I = eye(size(H));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

% Complimentary filter gain
alpha = 0.999;

for i = startSample:stopSample
    
    [accelReadings,~] = readAcceleration(imu);
    accel(i,:) = (accelReadings / 9.81) - [xOff,yOff,zOff]; % in G's
    
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
    
    % Normalizing (we care more about the direction than magnitude)
    
    MagX = MagX/fieldMagnitude;
    
    MagY = MagY/fieldMagnitude;
    
    MagZ = MagZ/fieldMagnitude;
    
    magField = [MagX; MagY; MagZ];
    
    %% To NED Frame
    GyroX = gyro(i,2);
    GyroY = gyro(i,1);
    GyroZ = -gyro(i,3);
    
    Gyro = [GyroX;GyroY;GyroZ];
    
    AccelX = accel(i,2);
    AccelY = accel(i,1);
    AccelZ = -accel(i,3);
    
     % Normalizing 
     
    accelMag = norm([AccelX AccelY AccelZ]);
    
    AccelX = AccelX/accelMag;
    AccelY = AccelY/accelMag;
    AccelZ = AccelZ/accelMag;
    
    if Theta == 1.5708  % Lazy way of avoiding gimbal lock (temporary)
        Theta = 1.5516;
    end 
    
    
    %% Converting Gyro to Euler rates
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
    zk = [AccelX; AccelY; AccelZ];
    
    % Measurement Model Accelerometer
    [ax,ay,az] = AccelModel(Xkp(1), Xkp(2));
    
    h_of_x = [ax;ay;az];
    
    H = MeasurementJacobian(Xkp(1), Xkp(2));
    
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
    
    %% Complimentary filter
    B = rotationMatrix(magField, Phi, Theta, Psi);
    
    mbx = B(1);
    mby = B(2);
    
    psi_comp = atan2(mby,mbx);
    
    yaw = (alpha*Psi) + (1-alpha)*psi_comp;
    
    % Store for plotting
    PhiKalman = [PhiKalman; Xk(1)];
    ThetaKalman = [ThetaKalman; Xk(2)];
    PsiKalman = [PsiKalman; yaw]; 
    
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

function H = MeasurementJacobian(phi,theta)

H = [0 -cos(theta) 0; 
    cos(theta)*cos(phi) -sin(theta)*sin(phi) 0; 
    -cos(theta)*sin(phi) -sin(theta)*cos(phi) 0];

end 

function [ax,ay,az] = AccelModel(Phi, Theta)

ax = - sin(Theta);

ay = cos(Theta)*sin(Phi);

az = cos(Theta)*cos(Phi);
end 

function B = rotationMatrix(A, phi, theta, psi)

X = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];

Y = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];

Z = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

R = (X*Y)*Z;

B = R*A;

 end 
