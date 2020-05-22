% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    Z_inv = inv(v2t(edge.measurement));
    e_const = t2v(Z_inv * (inv(x1) * x2));
    F_temp = e_const' * edge.information * e_const ;
    Fx += F_temp;
    

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    x_trans = v2t(x);
    R = x_trans(1:2,1:2);
    e_const = (R' * (l - x(1:2))) - edge.measurement;
    F_temp = e_const' * edge.information * e_const ;
    Fx += F_temp;

  end

end
