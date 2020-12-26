MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

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

% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

% Acceleration is also hidden - hard to initalize
AlphaX = 0; AlphaY = 0; AlphaZ = 0;

x0 = zeros(9,1);

x0(1) = ThetaX + OmegaX*dt + 0.5*AlphaX*dt^2;
x0(2) = ThetaY + OmegaY*dt + 0.5*AlphaY*dt^2;
x0(3) = ThetaZ + OmegaZ*dt + 0.5*AlphaZ*dt^2;
x0(4) = OmegaX + AlphaX*dt;
x0(5) = OmegaY + AlphaY*dt;
x0(6) = OmegaZ + AlphaZ*dt;
x0(7) = AlphaX;
x0(8) = AlphaY;
x0(9) = AlphaZ;
 
Wk = 0;  

xk = x0+Wk;

P0 = eye(length(x0));

beta = 2;
alpha = 0.1;

sigmas = sigmaPoints(xk,P0,alpha);

[Wc, Wm] = weights(xk,alpha,beta);

% creating column vectors for easier computation
sigmapoints = sigmas.';

%% Passing sigma points through x0 (or xk with noise)

% Transormed Sigma Points
propagatedPoints = secondOrderUKFPropagation(dt, sigmapoints, Wk);

%% Unscented Transform

% Find the mean of the transformed sigma points
yT = Wm*propagatedPoints; % row vector for now

Q = 0.001;

points = propagatedPoints.';

% Find the covariance of the transformed sigma points
P = PredictCovarianceUKF(x0, points, sigmas, yT ,Wc, Q);


% new sigma points

xPredict = yT.';

Newsigmas = sigmaPoints(xPredict,P, alpha);

% Newsigmas.' makes it easier to look at, each state has a value 

%% Measurements
 
