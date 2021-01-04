addpath('/Users/dariusmensah/Documents/CapstoneMonitoringHumanMovements/UnscentedFilter') 

P = eye(6);

states = [0;0;0;0.2253;-.1456;0.4119];

beta = 2;
kappa = 3-length(states);
alpha = 0.1;

%% First, gather sigma points

% The code accepts the states as a column vector (states x 1) like normal
% It then outputs a (states x sigma points) matrix
samplePoints = sigmaPoints(states,P,alpha); 

%% Then, pass the sigma points through your model (Prediction)
dt = 1/500;

% Input the epoch (dt), sigma Points, and noise (wk)
NewPoints = firstOrderUKFPropagation(dt, samplePoints, 0);

%% Compute the weights 
[Wc, Wm] = weights(NewPoints,alpha,beta);

%% Perform the Unscented Transform by summing the sample mean and covariances
%% With their respective weights to produce a new mean and covariance

Mu_x = NewPoints*Wm;

Px = PredictCovarianceUKF(NewPoints, samplePoints, Mu_x ,Wc, 0);

%% Measurements
% First get the new sigma points from the newly calculated mean and
% covariance

newSigmaPoints = sigmaPoints(Mu_x,Px,alpha);

% Use measurement equations


