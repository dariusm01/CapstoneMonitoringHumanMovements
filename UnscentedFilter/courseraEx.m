% One iteration of an unscented kalman filter

x0 = [0;5];

P0 = [0.01 0;
      0    1];
  
alpha = 0.99;

dt = 0.5;

sigmas = sigmaPoints(x0,P0,alpha);

F = [1 dt;
     0 1];
 
B = [0;dt];
u = -2;

sigmapoints = sigmas.';

propagatedPoints = zeros(size(sigmapoints));

j = size(sigmapoints);

for i = 1:j(2)
    propagatedPoints(:,i) = F*sigmapoints(:,i);
end 

propagatedPoints = propagatedPoints + B*u;

Wm = ones(1,length(sigmapoints));

Wm = Wm*(1/6);

Wm(1) = 1/3;

Wc = Wm;

points = propagatedPoints.';

%% Unscented Transform

x = Wm*points;

% Predicted covariance
col_vec_mean = x.';


Q = [0.1 0;
     0   0.1];

newCov = PredictCovarianceUKF(x0, propagatedPoints, sigmas, x ,Wc, Q);


%% Measurements

actualMeasurement = pi/6;

% new sigma points

Newsigmas = sigmaPoints(col_vec_mean,newCov,alpha);

z = zeros(length(Newsigmas),1);

S = 20;
D = 40;

% Using sigma points in measurement eq to get measurement sigma points

for j = 1:length(Newsigmas)
    z(j) = atan2(S, D-Newsigmas(j,1));
end 

mu_z = Wm*z;

% Measurement sigma point covariance

Pz = 0;

% Measurement noise covariance
R = 0.01; 
for u = 1:length(z)
    
    subtrt = z(u)-mu_z;
    
    Pz = Wc(u)*(subtrt*subtrt.') + Pz;
    
end 

Pz = Pz + R;


%% Cross covariance (prediction and measurments)

Pxz = zeros(size(x0));
for e = 1:length(z)
    
    predicX = (propagatedPoints(:,e)-col_vec_mean);
    
    measX = z(e)-mu_z;
    
    Pxz = Wm(e)*(predicX*measX.') + Pxz;
    
end 


%% Kalman Gain

K = Pxz*(Pz^-1);

%% Updating the state

xNew = col_vec_mean + K*(actualMeasurement-mu_z);

%% Updating the covariance

PNew = newCov - K*Pz*K.';