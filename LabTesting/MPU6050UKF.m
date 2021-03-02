% An example of the first iteration of the unscented kalman filter for 
% our project. 
%   The steps include:
%   1. Initializing the states and covariance
%   2. Calculate sigma points (cholesky decomp for matrix sqrt)
%   3. Prediction - > plug in sigma points into prediction equation
%   4. Calculate the weights Wm & Wc (mean and covariance)
%   5. Perform the Unscented Transform to get μx (prior) and Px
%   6. Measurements -> use the μx & Px to find new sigma points
%   7. Plug the new sigma points into the measurement model equation(s) 
%   8. Find μz (unscented transform)
%   9. Take measurement (z)
%  10. Find Pz (measurement covariance)
%  11. Calculate the cross - covariance (Pxz)
%  12. Determine the kalman gain (K)
%  13. Use the kalman gain to find the estimation (posterior)
%  14. Update the covariance 
%  15. Repeat for next time step

addpath("UnscentedFilter");

%% Getting Accelerometer Data (in NED frame)
mat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/z_AxisRotation.xlsx");

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

time = time.'*dt;

%% Initializing the states and covariance

states = [Phi; Theta; Psi];

P = 500*eye(length(states));

beta = 2;
kappa = 3-length(states);

% 0 ≤ α ≤ 1 
% Larger α spreads the sigma points further from the mean

alpha = 0.2;

% Noise
AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Wk = 0;

Qk = eye(size(P))*GyroSpectralDensity;

Rk = eye(size(P))*0.3;

%% Values we want to plot 

PhiKalman = [];
ThetaKalman = [];
PsiKalman = [];

for iii = 1:length(time)
    
    Gyro = [GyroX(iii);GyroY(iii);GyroZ(iii)];
    
    %% First, gather sigma points

    % The code accepts the states as a column vector (states x 1) like normal
    % It then outputs a (states x sigma points) matrix
    samplePoints = sigmaPoints(states,P,alpha); 

    %% Then, pass the sigma points through your model (Prediction)
    % Input the epoch (dt), sigma Points, the input (u), and noise (wk)
    NewPrediction = firstOrderUKFPropagation(samplePoints, dt, Gyro, Wk);
    
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

    %% Passing sigma points through non linear measurement model:
    % the measurement function converts the filter’s prior into a measurement
    
    % Measurement Model Accelerometer
    
    % |ax|     |    -sin(θ)    |
    % |ay|  =  |  cos(θ)sin(φ) |
    % |az|     |  cos(θ)cos(φ) |
 

    newMeasurementSigmaPoints = zeros(size(newSigmaPoints));
    
    for i = 1:length(newMeasurementSigmaPoints)
        newMeasurementSigmaPoints(1,i) = -sin(newSigmaPoints(2,i));
        newMeasurementSigmaPoints(2,i) = cos(newSigmaPoints(2,i))*sin(newSigmaPoints(1,i)); 
        newMeasurementSigmaPoints(3,i) = cos(newSigmaPoints(2,i))*cos(newSigmaPoints(1,i));
    end 

    Mu_z = newMeasurementSigmaPoints*Wm;

    %% Measurment covariance
    Pz = PredictCovarianceUKF(newMeasurementSigmaPoints, newSigmaPoints, Mu_z, Wc, Rk);

    % measurements from sensor
    % (1:3) = accelerometer
    sensorReadings = [AccelX(iii); AccelY(iii); AccelZ(iii)];
    z = sensorReadings;
    
    %% Cross Covariance
    Pxz = CrossCovariance(Mu_x, Mu_z, newSigmaPoints, newMeasurementSigmaPoints, Wc);

    %% Kalman Gain
    K = Pxz*pinv(Pz);

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
    PsiKalman = [PsiKalman; Xk(3)];
end 

q = angle2quat(PsiKalman,ThetaKalman,PhiKalman);
q = quatnormalize(q);

[yaw,pitch,roll] = quat2angle(q);

yaw = rad2deg(yaw);
pitch = rad2deg(pitch);
roll = rad2deg(roll);

%% Plotting

% figure(1)
% plot(time, wrapTo360(roll),'LineWidth',1)
% title('Roll Angle ${\phi}$','interpreter','latex')
% xlabel("Time(s)")
% ylabel("Radians")
% grid on
% legend("Unscented Kalman Filter \Phi")
% hold off
% 
% figure(2)
% plot(time, pitch,'LineWidth',1)
% title('Pitch Angle ${\theta}$','interpreter','latex')
% xlabel("Time(s)")
% ylabel("Radians")
% grid on
% legend("Unscented Kalman Filter \Theta")
% hold off

translations = zeros(size(q));
translations(:,end) = [];

% figure(3)
% grid on
% for jj = 1:length(q)
%     plotTransforms(translations(jj,:),q(jj,:),"InertialZDirection","down")
%     pause(0.00005)
% end

% x-axis = green
% y-axis = red
% z-axis = blue
