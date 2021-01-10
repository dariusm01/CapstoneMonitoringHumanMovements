MeasuredData = readtable("SampleData.xlsx"); 

addpath("UnscentedFilter")

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

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

%% Unscented Filter

% Initial Angle Values 
Phi = phi(1); Theta = theta(1); Psi = psi(1); 

% Initial Gyro Values 
PhiDot = phi_dot(1); ThetaDot = theta_dot(1); PsiDot = psi_dot(1); 

%% Initializing the states and covariance
P = 500*eye(6);

states = [Phi; Theta; Psi; PhiDot; ThetaDot; PsiDot];

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Qk = eye(length(P));

Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);

Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);

Wk = 0; 

Rk = eye(size(P))*0.3;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];

for iii = 1:length(time)
    %% First, gather sigma points

    % The code accepts the states as a column vector (states x 1) like normal
    % It then outputs a (states x sigma points) matrix
    samplePoints = sigmaPoints(states,P,alpha); 

    %% Then, pass the sigma points through your model (Prediction)
    % Input the epoch (dt), sigma Points, and noise (wk)
    NewPrediction = firstOrderUKFPropagation(dt, samplePoints, Wk);

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

    propagatedAccel = zeros(3,length(newSigmaPoints));

    %% Passing sigma points through non linear measurement model:
    % the measurement function converts the filter’s prior into a measurement

    % |ax|     |     sin(θy)     |
    % |ay|  =  | -cos(θy)sin(θx) |
    % |az|     |  cos(θx)cos(θy) |

    for i = 1:length(propagatedAccel)
        propagatedAccel(1,i) = sind(newSigmaPoints(2,i));
        propagatedAccel(2,i) = -cosd(newSigmaPoints(2,i))*sind(newSigmaPoints(1,i)); 
        propagatedAccel(3,i) = cosd(newSigmaPoints(1,i))*cosd(newSigmaPoints(2,i));
    end 

    % For gyro measurment model, I have already converted the (p,q,r) to
    % angular rates using simulink

    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));

    newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
    newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:end,:);  % gyro stays the same

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    % (1:3) = accelerometer, (4:6) = gyroscope
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii); phi_dot(iii); theta_dot(iii); psi_dot(iii)];
    z = sensorReadings;
    
    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*(Pz)^-1;

    %% Compute the posterior using the prior and measurement residual
    y = z-Mu_z;
    
    %% Update the state
    Xk = Mu_x + K*y;

    %% Update the covariance
    Pk = Px - K*(Pz)*K.';
    
    %% Repeat for next iteration
    states = Xk;
    P = Pk;
    
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
title("Complimentary Filter vs Unscented Kalman Filter \phi (Roll) [\alpha = 0.95]")
plot(time, PhiKalman)
legend("\thetaX Complimentary Filter", "\thetaX Unscented Kalman Filter")
%legend("\thetaX Complimentary Filter", "\thetaX Unscented Kalman Filter", "\thetaX Accelerometer")
hold off


figure(2)
plot(time, ThetaAngleComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter vs Unscented Kalman Filter \theta (Pitch) [\alpha = 0.95]")
plot(time, ThetaKalman)
legend("\thetaY Complimentary Filter", "\thetaY Unscented Kalman Filter")
%legend("\thetaY Complimentary Filter", "\thetaY Unscented Kalman Filter", "\thetaX Accelerometer")
hold off

