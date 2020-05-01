function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible

% TODO: compute mu
n = length(sigma_points);
k = (n-1) / 2;
mu = zeros(k,1);
sigma = zeros(k);
##for i = 1:n
##  mu += w_m(i) * sigma_points(:,i);
##end

mu = sigma_points * w_m';


% TODO: compute sigma
for i = 1:n
  sigma += w_c(i) * ((sigma_points(:,i) - mu) * (sigma_points(:,i) - mu)');
  
endfor
