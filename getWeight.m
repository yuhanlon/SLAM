%% 
% This function is for generating mean value and correlation value for Unscented Kalman Filter
%
% @author: Yuhan Long <yuhanlon@andrew.cmu.edu>

function [weight_mean,weight_cor] =getWeight(alpha,beta,kappa,n)
	% the parameter here is defend by the Unscented Kalman Filter 
	weight_mean = [ones(1,2*n)*(1/(2*alpha^2*(n+kappa))),1-n/(alpha^2*(n+kappa))];
	weight_cor = [ones(1,2*n)*(1/(2*alpha^2*(n+kappa))),2-n/(alpha^2*(n+kappa))-alpha^2+beta];
end
