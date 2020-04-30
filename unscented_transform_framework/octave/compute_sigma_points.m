function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);
sigma_points = zeros(n,2*n+1);

% TODO: compute all sigma points

sigma_points = mu;
mu_tp = mu;
sqrt_matrix = sqrtm((n+lambda) * sigma);

for i = 1:n
  positive = mu + sqrt_matrix(:,i);
  negative = mu - sqrt_matrix(:,i);
  sigma_points = [sigma_points,positive, negative]
endfor


% TODO compute weight vectors w_m and w_c

w_m = lambda/ (n + lambda);
w_c = w_m + (1 - alpha**2 + beta);

for i = 1: (2*n)
  value = 1 / (2 * (n + lambda));
  w_m = [w_m ,value];
  w_c = [w_c, value];


end
