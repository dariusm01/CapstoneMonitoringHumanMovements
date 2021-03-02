function sigmas = sigmaPoints(X,P,alpha)

% Van der Merwe scaled sigma point implementation

n = length(X);

kappa = 3-n;

lambda = (alpha^2 * (n + kappa)) - n;

%% Sigma Points

sigmas = zeros(2*n+1, n);

% Matrix Square root of P scaled by (n+lambda)
U = chol((n+lambda)*P); 

% First row is the mean (x)
sigmas(1,1:n) = X;

% Transposing to make the loop easier
x = X.';

    for k = 1:n
        sigmas(k+1,:) = x + U(k,:);
        sigmas(n+k+1,:) = x - U(k,:);
    end 
sigmas = sigmas.';
end

