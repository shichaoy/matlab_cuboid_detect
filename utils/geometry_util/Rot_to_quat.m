function [ quat ] = Rot_to_quat( R )
% rotation matrix --> [qx qy qz qw]

% for corner case, see  http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

    quat = zeros(1,4);
    quat(4) = 0.5 * sqrt(1 + R(1,1) + R(2,2) + R(3,3));
    quat(1) = (R(3,2)-R(2,3))/(4*quat(4));
    quat(2) = (R(1,3)-R(3,1))/(4*quat(4));
    quat(3) = (R(2,1)-R(1,2))/(4*quat(4));
   
end

