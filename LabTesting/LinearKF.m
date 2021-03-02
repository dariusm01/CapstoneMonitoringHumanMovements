
%% Getting Accelerometer Data (in NED frame)
mat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/x_AxisRotation.xlsx");

AccelX = mat.AccelY;
AccelY = mat.AccelX;
AccelZ = mat.AccelZ * -1;

GyroX = mat.GyroX;
GyroY = mat.GyroY;
GyroZ = mat.GyroZ;

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
Phi = 0;
Theta = 0;
Psi = 0;

dt = 1/100;

time = 1:length(AccelX);

time = time.';

%% Prediction 

% x = Fx + Gu + w

F = [1 0 0; 
     0 1 0; 
     0 0 1];
 
G = [dt 0 0; 
     0 dt 0; 
     0 0 dt];
 
zk = [0;0;0];

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
Rk = eye(size(Pk_1))*0.1;

H = eye(size(Pk_1));

I = eye(size(H));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

for i = 1:length(time)
    
    % Convert Gyro to Euler derivatives
    Gyro = [GyroX(i);GyroY(i);GyroZ(i)];
    
    [phiDot,thetaDot,psiDot] = EulerRate(Xk_1(1),Xk_1(2), Gyro);
    
    % Euler Rates are inputs into the system
    u =  [phiDot;thetaDot;psiDot]; 
    
    % Prior
    Xkp = F*Xk_1 + G*u + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
 
    AccelMeasure = [AccelX(i); AccelY(i); AccelZ(i)];
    ax = AccelMeasure(1); ay = AccelMeasure(2); az = AccelMeasure(3);
    
    % Measurement Model Accelerometer
    zk(1) = atan2(ay,(sqrt(ax^2+az^2)));
    zk(2) = atan2(-ax,(sqrt(ay^2+az^2)));
    zk(3) = atan2(sqrt(ax^2+az^2),az);
  
    % Innovation (Residual)
    yk = zk - Xkp;
    
    % Kalman Gain  
    K = Pkp*H.'*pinv(Sk);
    
    % Posterior 
    Xk = Xkp + K*yk;
    
    % Covariance Update
    Pk = (I - K*H)*Pkp*(I - K*H).' + (K*Rk*K.');
    
    % Redefining for next iteration
    Xk_1 = Xk;
    
    Pk_1 = Pk;
    
    % Store for plotting
    PhiKalman = [PhiKalman; Xk(1)];
    ThetaKalman = [ThetaKalman; Xk(2)];
    PsiKalman = [PsiKalman; Xk(3)];
    
end 

% T = table(rad2deg(PhiKalman),rad2deg(ThetaKalman),rad2deg(PsiKalman));
% 
% pth = "/Users/dariusmensah/Desktop/LabTesting/MPU_6050/KalmanFilter/z_AxisRotationFast.xlsx";
% 
% writetable(T,pth);
% 
% clearvars;
% clc;

q = angle2quat(PsiKalman,ThetaKalman,PhiKalman);
q = quatnormalize(q);

[yaw,pitch,roll] = quat2angle(q);

yaw = rad2deg(yaw);
pitch = rad2deg(pitch);
roll = rad2deg(roll);

translations = zeros(size(q));
translations(:,end) = [];

% figure(3)
% grid on
% for jj = 1:length(q)
%     plotTransforms(translations(jj,:),q(jj,:),"InertialZDirection","down")
%     pause(0.00005)
% end

% plotTransforms(translations,q)

% x-axis = green
% y-axis = red
% z-axis = blue

