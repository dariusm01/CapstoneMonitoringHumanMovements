
%arduinosetup();

%% Name the file to save
fileName = '/MagnetometerValuesCalibrated2.xlsx';

filePath = '/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/realTimeMatlabCode';

%% Sensor info
a = arduino();

% Port: '/dev/cu.usbmodem401'
% Board: 'Mega2560'

imu = mpu9250(a,'SamplesPerRead', 100);

startSample = 1;
stopSample = 2000;

mag = zeros(stopSample, 3);     % [µT]

MagX = zeros(stopSample, 1);
MagY = zeros(stopSample, 1);
MagZ = zeros(stopSample, 1);

[Offset, Scale] = CalibrateMag(imu);

fprintf("\nNow Gathering Data\n")


for i = startSample:stopSample
    [magReadings,~] = readMagneticField(imu);
    mag(i,:) = magReadings;
    
    MagX(i) = mag(i,1) - Offset(1); % Hard Iron Correction
    MagX(i) = MagX(i)*Scale(1); % Soft Iron Correction
    
    MagY(i) = mag(i,2) - Offset(2);
    MagY(i) = MagY(i)*Scale(2);
    
    MagZ(i) = mag(i,3) - Offset(3);
    MagZ(i) = MagZ(i)*Scale(3);
    
    subplot(3,1,1);
    grid on
    plot(MagX)
    title("X-Axis Rotation")
    
    subplot(3,1,2);
    plot(MagY)
    title("Y-Axis Rotation")
   
    subplot(3,1,3);
    plot(MagZ)
    title("Z-Axis Rotation")
    
end 
    
fprintf("Storing Data\n")

T = table(MagX, MagY, MagZ);

%% Creating excel sheet 
ExportSheet(fileName, filePath, T);

%clearvars; 

fprintf("\nData Exported\n")

% plot3(MagX,MagY,MagZ)

r = [mean(MagX) mean(MagY) mean(MagZ)];

FieldStrength = norm(r); % Earth ranges from  25 µT to 65 µT

    
function ExportSheet(fileName, filePath, table)

fileToSave = strcat(filePath, fileName);

writetable(table, fileToSave);

end 

function [Offsets, Scale] = CalibrateMag(imu)

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