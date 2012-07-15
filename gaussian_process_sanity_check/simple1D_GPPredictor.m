%% Function to perform Gaussian Process Prediction 
% following Algorithm 2.1 on pg 19 of [Rasmussen, Williams 2006]
% Notations also follow the book.
%
% INPUTS:
% 1. X : inputs ( design matrix ) in training set
% 2. y : targets in training set
% 3. hyperparams: [length_scale sigma_f sigma_noise]
% 4. x_star: test input vector
%
% OUTPUTS:
% 1. predictive_mean
% 2. predictive_variance
% 3. Failcode = 0  : everything is ok
%    Failcode = -1 : trouble in cholesky decomposition (positive definite-ness
%    of K(X,X))
function [predictive_mean, predictive_variance, failcode] = simple1D_GPPredictor(X, y, hyperparams, x_star)

    length_scale = hyperparams(1,1);
    sigma_f = hyperparams(1,2);
    sigma_noise = hyperparams(1,3);
    
    failcode = 0;
    
    [K, PD_status] = computeCovMatrix(X,X, hyperparams);
    I = eye(size(K));
    
    % Detect when K matrix is ill-conditioned for cholesky decomposition
    if (PD_status == -1)
        disp('FAILCODE: K is not positive definite');
        predictive_mean = 0;
        predictive_variance = 0;
        failcode = -1;
        return;
    end
    
    L = chol(K + (sigma_noise^2)*I,'lower'); 
    
    % Computing Predictive Mean
    alpha = (L')\(L\y);        
    [k_star, PD_status] = computeCovMatrix(X,x_star, hyperparams);
    predictive_mean = (k_star')*alpha;
    
    % Computing Predictive Variance
    v = L\k_star;
    [k_star_star, PD_status] = computeCovMatrix(x_star, x_star, hyperparams);
    predictive_variance = k_star_star - (v')*v;
end

%% Function to compute the covariance matrix of 2 vectors X1 and X2
% For now, it uses Squared Exponential as the covariance function
% OUTPUT:
% 1. K : the covariance matrix
% 2. PD_status: reports on the positive-definiteness of the K matrix, if K
%                is a square matrix. If PD_statis is -1 then K is not
%                Positive-definite
function [K, PD_status] = computeCovMatrix(X1, X2, hyperparams)

    length_scale = hyperparams(1,1);
    sigma_f = hyperparams(1,2);
    sigma_noise = hyperparams(1,3);
    
    PD_status = 0;
    K = zeros(size(X1,2), size(X2,2));
    
    for i=1:size(X1,2)
        for j=1:size(X2,2)
            % Squared exponential Covariance function           
            diff = X1(:,i) - X2(:,j);
            sqr_dist = 0;
            for n=1:size(diff,1)
                sqr_dist = sqr_dist + diff(n,1)*diff(n,1);
            end
            
            if (i==j)
                kronecker_delta = 1;
            else
                kronecker_delta = 0;
            end
            
            % Formula on page 19 of Rasmussen
            K(i,j) = ((sigma_f)^2)*exp((-0.5/(length_scale^2))*sqr_dist) + (sigma_noise^2)*kronecker_delta;
        end
    end
    
    % Check positive definiteness of K matrix, if K is a square matrix
    if (size(K,1) == size(K,2))   
        determinants = zeros(size(X1,2),1);        
        
        for i=1:size(X1,2)
            determinants(i,1) = det( K(1:i, 1:i) );
            if (determinants(i,1) < 0)
                PD_status = -1;
                disp('---NEGATIVE DETERMINANTS OF SYMMETRIC K FOUND');
                break;
            end
        end

    end   
end