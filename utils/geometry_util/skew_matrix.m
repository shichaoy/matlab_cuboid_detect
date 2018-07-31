function [ cp ] = skew_matrix( vec )
%UNTITLED20 Summary of this function goes here
%   Detailed explanation goes here

cp = [0         -vec(3) vec(2);
      vec(3)    0       -vec(1);
      -vec(2)   vec(1)  0];

end

