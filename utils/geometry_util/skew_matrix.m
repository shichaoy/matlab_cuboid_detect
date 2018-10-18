function [ cp ] = skew_matrix( vec )
% skew matrix for vector3

cp = [0         -vec(3) vec(2);
      vec(3)    0       -vec(1);
      -vec(2)   vec(1)  0];

end

