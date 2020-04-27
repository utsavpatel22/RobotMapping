function [p] = t2v(M)
  angle = atan(M(2,1) / M(1,1));
  p = [M(1,3); M(2,3); angle];
end
