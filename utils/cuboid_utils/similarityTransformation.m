function Tr = similarityTransformation(object)
% get 4*4 similarity transform [sR t;0 1]. from cuboid frame to world frame.

% rotY is object yaw angle.
R = [cos(object.rotY) -sin(object.rotY) 0; sin(object.rotY) cos(object.rotY) 0; 0 0 1];

% scale matrix
S = diag(object.scale);

if (size(object.pos,2)==3) % row vec
    Tr = [R*S object.pos'; 0 0 0 1];
else  % column vec
    Tr = [R*S object.pos; 0 0 0 1];
end