port = '/dev/cu.usbserial-AB0L9PP9';
board = 'Nano3';
a = arduino(port,board);

imu = mpu9250(a,'SamplesPerRead', 100);

startSample = 1;
stopSample = 3000;

mag = zeros(stopSample, 3); % [µT]

Mx = zeros(stopSample, 1);
My = zeros(stopSample, 1);
Mz = zeros(stopSample, 1);

[Offset, Scale] = CalibrateMag(imu);
fprintf("\n")

F_M = eye(3);

I_M = F_M;

Pk_1_m = eye(length(F_M))*500;

% Lambda function
field = @(F) sqrt((F^2)/3);

% 25 <= Earth field <= 65 µT

earthField = field(45);

% Initial estimation
xk_1_m = ones(3,1) * earthField;

% Measurment Noise Covariance
Rk_m = eye(length(F_M))*0.085;

% Process Noise Covariance
Qk_m = eye(length(F_M))*0.001;

% Observation Matrix

H_M = F_M;

%% Values we want to plot 

fieldX = [];
fieldY = [];
fieldZ = [];

fprintf("Performing state estimation")

for i = startSample:stopSample
    
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
    
    % Prior
    xk_p_M = F_M*xk_1_m;
    
    % Proccess Covariance matrix
    Mk = F_M*Pk_1_m*F_M.' + Qk_m;  
    
    % Innovation Covariance
    Sk_M = H_M*Mk*H_M.' + Rk_m;
    
    % Measurement
    zk_M = [MagX;MagY;MagZ];
    
    % Residual
    yk_M = zk_M - H_M*xk_p_M;
    
    % Kalman Gain  
    K_M = Mk*H_M.'*pinv(Sk_M);
    
    % Posterior 
    xk_M = xk_p_M + K_M*yk_M;
    
    % Covariance Update
    Pk_M = (I_M - K_M*H_M)*Mk*(I_M - K_M*H_M).' + (K_M*Rk_m*K_M.');
    
    % Store for plotting
    fieldX = [fieldX; xk_M(1)];
    fieldY = [fieldY; xk_M(2)];
    fieldZ = [fieldZ; xk_M(3)];
    
    % Redefining for next iteration
    xk_1_m = xk_M;
    
    Pk_1_m = Pk_M;
    
end 

% field = [fieldX fieldY fieldZ];
% 
% magnetometer = [Mx My Mz];

figure(1)
plot(Mx)
hold on
plot(fieldX)
title("Magnetic Field X-Axis")
ylabel("Microtesla [µT]")
legend("Sensor","Kalman Filter")
hold off

figure(2)
plot(My)
hold on
plot(fieldY)
title("Magnetic Field Y-Axis")
ylabel("Microtesla [µT]")
legend("Sensor","Kalman Filter")
hold off

figure(3)
plot(Mz)
hold on
plot(fieldZ)
title("Magnetic Field Z-Axis")
ylabel("Microtesla [µT]")
legend("Sensor","Kalman Filter")
hold off

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