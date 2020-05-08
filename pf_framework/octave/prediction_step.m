function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);

for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  particles(i).pose(1) = particles(i).pose(1) + normrnd(u.t, transNoise) * cos(normalize_angle(particles(i).pose(3) + normrnd(u.r1, r1Noise)));
  particles(i).pose(2) = particles(i).pose(2) + normrnd(u.t, transNoise) * sin(normalize_angle(particles(i).pose(3) + normrnd(u.r1, r1Noise)));
  particles(i).pose(3) = normalize_angle(particles(i).pose(3) + normalize_angle(normrnd(u.r1, r1Noise) + normrnd(u.r2, r1Noise))); 
  

end

end
