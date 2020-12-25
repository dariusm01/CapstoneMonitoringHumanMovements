function newCov = PredictCovarianceUKF(x0, propagatedPoints, sigmas, yT ,Wc, Q)

% P = zeros(length(x0));
%     for k = 1:length(sigmas)
% 
%         y = sigmas(k,:) - yT;
% 
%         P = Wc(k)*outerproduct(y,y) + P;
%     end
% 
% % adding process noise
% newCov  = P+Q;


% Predicted covariance
col_vec_mean = yT.';

P = zeros(length(x0));

    for i = 1:length(sigmas)

        y = propagatedPoints(:,i)- col_vec_mean;
        P = (Wc(i)*(y*y.'))+P;
    end 
    
newCov = P+Q;
end 