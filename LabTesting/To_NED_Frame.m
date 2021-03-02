mat = readtable("z_AxisRotationFast.xlsx");

GyroX = mat.GyroX;
GyroY = mat.GyroY;
GyroZ = mat.GyroZ;

[GyroY,GyroX] = swap(GyroX, GyroY);

T = table(GyroX, GyroY, -GyroZ);

writetable(T,"/Users/dariusmensah/Desktop/LabTesting/MPU_6050/NED_Frame/z_AxisRotationFast.xlsx");

clearvars;
clc;