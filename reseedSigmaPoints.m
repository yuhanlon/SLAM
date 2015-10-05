%%
% this is the sigma point reseeding process in Unscented Kalman Filter 
%
% @author: Yuhan Long <yuhuanlon@andrew.cmu.edu>

function [sp] = reseedSigmaPoints(x_aug,P_state_aug,alpha,beta,kappa)
	% the parameters are defined according to the Unscented Kalman Filter
	n = size(x_aug,1);
	zeta = sqrt(n+kappa)*alpha;
	
	sp(:,2*n+1) = x_aug;
	sp(:,1:n) = bsxfun(@plus,x_aug,zeta*chol(P_state_aug)');
	sp(:,n+1:2*n) = bsxfun(@minus,x_aug,zeta*chol(P_state_aug)'); 

end
