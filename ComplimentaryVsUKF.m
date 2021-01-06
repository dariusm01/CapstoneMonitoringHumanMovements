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

addpath("UnscentedFilter")

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


%% Unscented Kalman Filter

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

%% Initializing the states and covariance
P = 500*eye(6);

states = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ];

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

AngleXKalman = [];
AngleYKalman = [];

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

    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | -cos(θx)cos(θy)|
    
    % Using the positive version for az since the outout should be +1g
    
    % |ax|     | cos(θx)sin(θy) |
    % |ay|  =  |     sin(θx)    |
    % |az|     | cos(θx)cos(θy) | 

    for i = 1:length(propagatedAccel)
        propagatedAccel(1,i) = cosd(newSigmaPoints(1,i))*sind(newSigmaPoints(2,i));
        propagatedAccel(2,i) = sind(newSigmaPoints(2,i)); 
        propagatedAccel(3,i) = cosd(newSigmaPoints(1,i))*cosd(newSigmaPoints(2,i));
    end 

    % For gyro measurment model, it would be best to use
    % ω_true = ω_output - bias
    % bias is not calclated, so I am skipping it for now

    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));

    newMeasurementSigmaPoints(1:3,:) = propagatedAccel;
    newMeasurementSigmaPoints(4:end,:) = newSigmaPoints(4:end,:);  % gyro stays the same

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    % (1:3) = accelerometer, (4:6) = gyroscope
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii); GyroX(iii); GyroY(iii); GyroZ(iii)];
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
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
end 


figure(1)
plot(time, AngleXComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter vs Unscented Kalman Filter \thetaX (Roll) [\alpha = 0.95]")
plot(time, AngleXKalman)
%plot(time, AccelAngleX)
legend("\thetaX Complimentary Filter", "\thetaX Unscented Kalman Filter")
%legend("\thetaX Complimentary Filter", "\thetaX Unscented Kalman Filter", "\thetaX Accelerometer")
hold off


figure(2)
plot(time, AngleYComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter vs Unscented Kalman Filter \thetaY (Pitch) [\alpha = 0.95]")
plot(time, AngleYKalman)
% plot(time, AccelAngleY)
legend("\thetaY Complimentary Filter", "\thetaY Unscented Kalman Filter")
%legend("\thetaY Complimentary Filter", "\thetaY Unscented Kalman Filter", "\thetaX Accelerometer")
hold off
