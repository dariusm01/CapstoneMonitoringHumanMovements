function sigmas = sigmaPoints(X,P,beta,alpha)

% Van der Merwe scaled sigma point implementation

n = length(X);

kappa = 3-n;

%% Weights

lambda = (alpha^2 * (n + kappa)) - n;

Wc = zeros(1,(2*n+1));

for i = 1:(2*n+1)
    Wc(i) = 1 / (2*(n + lambda));
end 

% Besides the first, the weights for the covariance and mean are the same
Wm = Wc;

% Changing the first weights for the covariance and mean
Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);
Wm(1) = lambda / (n + lambda);


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

end

