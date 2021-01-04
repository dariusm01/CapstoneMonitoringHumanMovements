function newCov = PredictCovarianceUKF(propagatedPoints, sigmas, newPoints ,Wc, Q)
sigmas = sigmas.';
dim = size(sigmas);
x0 = dim(2);
% Predicted covariance
col_vec_mean = newPoints.';

P = zeros(length(x0));

    for i = 1:length(sigmas)

        y = propagatedPoints(:,i)- col_vec_mean;
        P = (Wc(i)*(y*y.'))+P;
    end 
    
newCov = P+Q;
end 