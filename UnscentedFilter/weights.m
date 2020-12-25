function [Wc, Wm] = weights(X,alpha,beta)

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
end 