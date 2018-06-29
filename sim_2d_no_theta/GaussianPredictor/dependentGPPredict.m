%% Function to perform Gaussian Process Prediction 
% following Phillip Boyle and Marcus Frean's paper:
% 'DEPENDENT GAUSSIAN PROCESSES' (2004)
% 
% INPUTS:
% 1. X : inputs ( design matrix ) in training set with n data points
%    [ x(1)	    x(2)	...    x(n)
%      y(1)     y(2)    ...    y(n)
%      th(1)    th(2)   ...    th(n) ];

% 2. y : targets in training set, index from 1..n is the entry of the
% training set:
%      out1      out2        out3        out4
%
%    [ y_1(1)   y_2(1)      y_3(1)      y_4(1)
%      y_1(2)   y_2(2)      y_3(2)      y_4(2)
%       ...      ...         ...         ...
%      y_1(n)   y_2(n)      y_3(n)      y_4(n)]
%
%
% 3. hyperparams: gp_hyperparams = [hp_v; hp_w; hp_A; hp_B; hp_sigma;
% hp_mu];
% 4. x_star: test input vector
%    x_star  = [ x_star(1);
%                x_star(2);
%                x_star(3)];
%
% OUTPUTS:
% 1. predictive_mean
% 2. predictive_variance
% 3. Failcode = 0  : everything is ok
%    Failcode = -1 : trouble in cholesky decomposition (positive definite-ness
%    of K(X,X))
function [predictive_mean, predictive_variance, failcode] = dependentGPPredict(X, y, hyperparams, x_star)

    hp_v = hyperparams(1,:);
    hp_w = hyperparams(2,:);
    hp_A = hyperparams(3,:);
    hp_B = hyperparams(4,:);
    hp_sigma = hyperparams(5,:);
    hp_mu = hyperparams(6,:);

    failcode = 0;

    % Calculate covariance matrix
    K = computeCovMatrix(X,X, hyperparams);

    % Use Cholesky for efficient inverse
    L = chol(K,'lower'); 

    % Compute k(x_star, X)
    kStar = computeCovMatrix(x_star, X, hyperparams);

    yDependent = [y(:,1); y(:,2); y(:,3); y(:,4)];


    % Computing Predictive Mean
    alpha = (L')\(L\yDependent);   
    predictive_mean(1,1) = kStar(1,:)*alpha;
    alpha = (L')\(L\yDependent);   
    predictive_mean(2,1) = kStar(2,:)*alpha;
    alpha = (L')\(L\yDependent);   
    predictive_mean(3,1) = kStar(3,:)*alpha;
    alpha = (L')\(L\yDependent);   
    predictive_mean(4,1) = kStar(4,:)*alpha;
	
	
    % Computing Predictive Variance
    % v = L\k_star;
    % [k_star_star, PD_status] = computeCovMatrix(x_star, x_star, hyperparams);
    % predictive_variance = k_star_star - (v')*v;

	predictive_variance = 0;
end

%% Function to compute the covariance matrix of 2 vectors X1 and X2
% For now, it uses Squared Exponential as the covariance function
% OUTPUT:
% 1. K : the covariance matrix
function K = computeCovMatrix(X1, X2, hyperparams)

	hp_v = hyperparams(1,:);
	hp_w = hyperparams(2,:);
	hp_A = hyperparams(3,:);
	hp_B = hyperparams(4,:);
	hp_sigma = hyperparams(5,:);
	hp_mu = hyperparams(6,:);
    	
	% Assemble the Covariance matrix from smaller blocks
	Ksub11 = computeSubCovarianceMatrix(X1, X2, hyperparams, 1, 1);
    Ksub12 = computeSubCovarianceMatrix(X1, X2, hyperparams, 1, 2);
	Ksub13 = computeSubCovarianceMatrix(X1, X2, hyperparams, 1, 3);
	Ksub14 = computeSubCovarianceMatrix(X1, X2, hyperparams, 1, 4);

	Ksub21 = computeSubCovarianceMatrix(X1, X2, hyperparams, 2, 1);
    Ksub22 = computeSubCovarianceMatrix(X1, X2, hyperparams, 2, 2);
	Ksub23 = computeSubCovarianceMatrix(X1, X2, hyperparams, 2, 3);
	Ksub24 = computeSubCovarianceMatrix(X1, X2, hyperparams, 2, 4);
	
	Ksub31 = computeSubCovarianceMatrix(X1, X2, hyperparams, 3, 1);
    Ksub32 = computeSubCovarianceMatrix(X1, X2, hyperparams, 3, 2);
	Ksub33 = computeSubCovarianceMatrix(X1, X2, hyperparams, 3, 3);
	Ksub34 = computeSubCovarianceMatrix(X1, X2, hyperparams, 3, 4);
	
	Ksub41 = computeSubCovarianceMatrix(X1, X2, hyperparams, 4, 1);
    Ksub42 = computeSubCovarianceMatrix(X1, X2, hyperparams, 4, 2);
	Ksub43 = computeSubCovarianceMatrix(X1, X2, hyperparams, 4, 3);
	Ksub44 = computeSubCovarianceMatrix(X1, X2, hyperparams, 4, 4);
	
	K = [Ksub11, Ksub12, Ksub13, Ksub14;
         Ksub21, Ksub22, Ksub23, Ksub24;
		 Ksub31, Ksub32, Ksub33, Ksub34;
		 Ksub41, Ksub42, Ksub43, Ksub44];
end

% Function to compute the subblock of Covariance matrix
% Equation (1) in Phillip & Boyle's paper
function Ksub = computeSubCovarianceMatrix(X1, X2, hyperparams, subblock_i, subblock_j)

	hp_v = hyperparams(1,:);
	hp_w = hyperparams(2,:);
	hp_A = hyperparams(3,:);
	hp_B = hyperparams(4,:);
	hp_sigma = hyperparams(5,:);
	hp_mu = hyperparams(6,:);
	
	Ksub = zeros(size(X1,2), size(X2,2));
	
	p = 1;	
	
	if (subblock_i > subblock_j)		
		for id_X1=1:size(X1,2)
			for id_X2=1:size(X2,2)				
				% Compute d, the distance between 2 entries in input space
				d = X1(:,id_X1) - X2(:,id_X2);		
				
				% Apply translation to d, as described in [Boyle, Frean]
				d_trans = d + hp_mu(1,1:size(d,1))';
				
				% Compute length scales
				Ai = hp_A(1,subblock_i).*eye(size(d,1));
				Aj = hp_A(1,subblock_j).*eye(size(d,1));
				
				% Compute | A_1 + A_2 |
				normA = hp_A(1,subblock_i) + hp_A(1,subblock_j);
				
				% Compute A_1 ( A_1 + A_2 )^-1 A_2
				sumA = Ai*inv(Ai+Aj)*Aj;				
				
				coef = ((((2*pi)^(p/2))*(hp_v(1,subblock_i)*hp_v(1,subblock_j)))/sqrt(normA));				
														
				K_U = coef*exp(-0.5.*((d_trans')*sumA*(d_trans)));
				
				Ksub(id_X1,id_X2) = K_U;
			end
		end
	elseif (subblock_i < subblock_j)
		for id_X1=1:size(X1,2)
			for id_X2=1:size(X2,2)
				% Compute d, the distance between 2 entries in input space
				d = X1(:,id_X1) - X2(:,id_X2);	
				
				% Apply translation to d, as described in [Boyle, Frean]
				d_trans = d - hp_mu(1,1:size(d,1))';
				
				% Compute length scales
				Ai = hp_A(1,subblock_i).*eye(size(d,1));
				Aj = hp_A(1,subblock_j).*eye(size(d,1));
				
				% Compute | A_1 + A_2 |
				normA = hp_A(1,subblock_i) + hp_A(1,subblock_j);
				
				% Compute A_1 ( A_1 + A_2 )^-1 A_2
				sumA = Ai*inv(Ai+Aj)*Aj;
								
				coef = ((((2*pi)^(p/2))*(hp_v(1,subblock_i)*hp_v(1,subblock_j)))/sqrt(normA));
										
				
				K_U = coef*exp(-0.5.*((d_trans')*sumA*(d_trans)));
				
				Ksub(id_X1,id_X2) = K_U;
			end
		end
	elseif (subblock_i == subblock_j)
		for id_X1=1:size(X1,2)
			for id_X2=1:size(X2,2)
				% Compute d, the distance between 2 entries in input space
				d = X1(:,id_X1) - X2(:,id_X2);
				
				% Compute length scales
				A = hp_A(1,subblock_i).*eye(size(d,1));
				B = hp_B(1,subblock_i).*eye(size(d,1));
				
				% Compute |A|, |B|
				normA = hp_A(1,subblock_i);
				normB = hp_B(1,subblock_i);
				
				K_U = (((pi^(p/2))*(hp_v(1,subblock_i)^2))/sqrt(normA))*exp(-0.25.*((d')*A*(d)));
				K_V = (((pi^(p/2))*(hp_w(1,subblock_i)^2))/sqrt(normB))*exp(-0.25.*((d')*B*(d)));
				
				if (id_X1==id_X2)
					kronecker_delta = 1;
				else
					kronecker_delta = 0;
				end
				
				K_Y = K_U + K_V + kronecker_delta*hp_sigma(1,subblock_i)^2;

				Ksub(id_X1,id_X2) = K_Y;
			end
		end		
	end
end




