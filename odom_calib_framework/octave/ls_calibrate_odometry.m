% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 

  % TODO: initialize H and b of the linear system
  info_mat = eye(3);
  H = zeros(9);
  b = zeros(9,1);
  
  
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  
  for i = 1:size(Z,1)
    e = error_function(i, X, Z);
    J = jacobian(i, Z);
    b += J' * info_mat * e;
    H += J' * info_mat * J;
  endfor

  % TODO: solve and update the solution
  R = chol(H);
  y = -1*inv(R') * b;
  X_delta = inv(R) * y;
  X_delta = reshape(X_delta,3,3)';
  X += X_delta;
  
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  e = Z(i, 1:3)' - X * Z(i, 4:6)';
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  J = zeros(3,9);
  J(1,1) = J(2,4) = J(3,7) = -1 * Z(i,4);
  J(1,2) = J(2,5) = J(3,8) = -1 * Z(i,5);
  J(1,3) = J(2,6) = J(3,9) = -1 * Z(i,6);
end
