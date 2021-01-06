MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500; 
 
% Initial Angle Values (guess)
ThetaX = 0; ThetaY = 0;

%% Values we want to plot 

AngleXComplimentary = [];
AngleYComplimentary = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.95;

%% Complimentary Filter

for i = 1:length(time)

    % angle corrections using accelerometer 
    ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 

    ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  
    
    newAngleX = alpha*(GyroX(i)*dt+ThetaX) + (1-alpha)*ThetaXAccel;
    
    newAngleY = alpha*(GyroY(i)*dt+ThetaY) + (1-alpha)*ThetaYAccel;
    
    
    % Store for plotting
    AngleXComplimentary = [AngleXComplimentary;newAngleX];
    AngleYComplimentary = [AngleYComplimentary;newAngleY];
    
    AccelAngleX = [AccelAngleX; ThetaXAccel];
    AccelAngleY = [AccelAngleY; ThetaYAccel];
    
    
    ThetaX = newAngleX;
    
    ThetaY = newAngleY;

end 


%% Extended Kalman Filter

% x = Fx + w

F = [1 0 0 dt 0 0; 
     0 1 0 0 dt 0; 
     0 0 1 0 0 dt; 
     0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1];
 
Wk = 0; 
 
% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

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

AngleXKalman = [];
AngleYKalman = [];

for j = 1:length(time)

    % Prior
    Xkp = F*Xk_1 + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
    
    zk = [AccelX(j); AccelY(j); AccelZ(j); GyroX(j); GyroY(j); GyroZ(j)];  
    
    %% Jacobian (partial derivatives)
    H(1,1) = -sind(Xkp(1))*sind(Xkp(2));
    H(2,1) = cosd(Xkp(1));
    H(3,1) = -sind(Xkp(1))*cosd(Xkp(2));
    
    H(1,2) = cosd(Xkp(1))*cosd(Xkp(2));
    H(2,2) = 0;
    H(3,2) = -cosd(Xkp(1))*sind(Xkp(2));
    
    H(3,3) = 0;
    
    % Measurement Model 
    % the measurement function (h_of_x) converts the filter’s prior into a measurement
    % Using the positive version for az since the outout should be +1g
    
    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | cos(θx)cos(θy) | 
    
    h_of_x = [cosd(Xkp(1))*sind(Xkp(2));
              sind(Xkp(1));
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
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
end 


figure(1)
plot(time, AngleXComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter vs Extended Kalman Filter \thetaX (Roll) [\alpha = 0.95]")
plot(time, AngleXKalman)
%plot(time, AccelAngleX)
legend("\thetaX Complimentary Filter", "\thetaX Extended Kalman Filter")
%legend("\thetaX Complimentary Filter", "\thetaX Extended Kalman Filter", "\thetaX Accelerometer")
hold off


figure(2)
plot(time, AngleYComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter vs Extended Kalman Filter \thetaY (Pitch) [\alpha = 0.95]")
plot(time, AngleYKalman)
% plot(time, AccelAngleY)
legend("\thetaY Complimentary Filter", "\thetaY Extended Kalman Filter")
%legend("\thetaY Complimentary Filter", "\thetaY Extended Kalman Filter", "\thetaX Accelerometer")
hold off
