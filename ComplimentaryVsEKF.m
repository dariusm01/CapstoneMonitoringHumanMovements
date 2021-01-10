MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500; 
 
AngleSim = sim("RateGyroUsingQuaternions.slx");

% Outputs 57x1
phi = AngleSim.phi.signals.values;
theta = AngleSim.theta.signals.values;
psi = AngleSim.psi.signals.values;
phi_dot = AngleSim.phi_dot.signals.values;
theta_dot = AngleSim.theta_dot.signals.values;
psi_dot = AngleSim.psi_dot.signals.values;

%% Resampling to get 50x1
% resamples the input sequence, x, at 7/8 times the original sample rate
% 57*(7/8) = 49.8750 -> ceil(49.8750) = 50
phi = resample(phi,7,8);
theta = resample(theta,7,8);
psi = resample(psi,7,8);
phi_dot = resample(phi_dot,7,8);
theta_dot = resample(theta_dot,7,8);
psi_dot = resample(psi_dot,7,8);

%% Initial Values 
Phi = phi(1);
Theta = theta(1);

%% Values we want to plot 

PhiAngleComplimentary = [];
ThetaAngleComplimentary  = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.95;

%% Complimentary Filter

for i = 1:length(time)

    % angle corrections using accelerometer 
    PhiAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 

    ThetaAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  
    
    newAngleX = alpha*(phi_dot(i)*dt+Phi) + (1-alpha)*PhiAccel;
    
    newAngleY = alpha*(theta_dot(i)*dt+Theta) + (1-alpha)*ThetaAccel;
    
    
    % Store for plotting
    PhiAngleComplimentary = [PhiAngleComplimentary; newAngleX];
    ThetaAngleComplimentary  = [ThetaAngleComplimentary ;newAngleY];
    
    AccelAngleX = [AccelAngleX; PhiAccel];
    AccelAngleY = [AccelAngleY; ThetaAccel];
    
    
    Phi = newAngleX;
    
    Theta = newAngleY;

end 


%% Extended Filter

% Initial Angle Values 
Phi = phi(1); Theta = theta(1); Psi = psi(1); 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
PhiDot = phi_dot(1); ThetaDot = theta_dot(1); PsiDot = psi_dot(1); 

% State Matrix
Xk_1 = [Phi; Theta; Psi; PhiDot; ThetaDot; PsiDot]; 

% Prediction 

% x = Fx + w

F = [1 0 0 dt 0 0; 
     0 1 0 0 dt 0; 
     0 0 1 0 0 dt; 
     0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1];
 
Wk = 0; 

% Covariance Matrix 

Pk_1 = eye(length(F))*500;

AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Qk = eye(length(F));

Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);

Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);

% Measurement noise
Rk = eye(size(Pk_1))*0.1;

H = eye(size(Pk_1));

I = eye(size(H));

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];

for i = 1:length(time)

    % Prior
    Xkp = F*Xk_1 + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
    % the measurement function (H) converts the filter’s prior into a measurement
    
    zk = [AccelX(i); AccelY(i); AccelZ(i); phi_dot(i); theta_dot(i); psi_dot(i)];  
    
    %% Jacobian 
    H(1,1) = -sind(Xkp(1))*sind(Xkp(2));
    H(2,1) = cosd(Xkp(1));
    H(3,1) = -sind(Xkp(1))*cosd(Xkp(2));
    
    H(1,2) = cosd(Xkp(1))*cosd(Xkp(2));
    H(2,2) = 0;
    H(3,2) = -cosd(Xkp(1))*sind(Xkp(2));
    
    H(3,3) = 0;
    
    % Measurement Model 
    
    % |ax|     |     sin(θy)     |
    % |ay|  =  | -cos(θy)sin(θx) |
    % |az|     |  cos(θx)cos(θy) |
 
    % For gyro measurment model, I have already converted the (p,q,r) to
    % angular rates using simulink
    
    h_of_x = [sind(Xkp(2));
              -cosd(Xkp(2))*sind(Xkp(1));
              cosd(Xkp(1))*cosd(Xkp(2));
              Xkp(4);
              Xkp(5);
              Xkp(6)];
    
    % Innovation (Residual)
    yk = zk - h_of_x;
    
    % Kalman Gain  
    K = Pkp*H.'*(Sk^-1);
    
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
end 


%% Plotting

figure(1)
plot(time, PhiAngleComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter vs Extended Kalman Filter \phi (Roll) [\alpha = 0.95]")
plot(time, PhiKalman)
legend("\thetaX Complimentary Filter", "\thetaX Extended Kalman Filter")
%legend("\thetaX Complimentary Filter", "\thetaX Extended Kalman Filter", "\thetaX Accelerometer")
hold off


figure(2)
plot(time, ThetaAngleComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter vs Extended Kalman Filter \theta (Pitch) [\alpha = 0.95]")
plot(time, ThetaKalman)
legend("\thetaY Complimentary Filter", "\thetaY Extended Kalman Filter")
%legend("\thetaY Complimentary Filter", "\thetaY Extended Kalman Filter", "\thetaX Accelerometer")
hold off

