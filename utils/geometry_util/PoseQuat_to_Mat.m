function T = PoseQuat_to_Mat(pose_quat_v)
% [x y z qx qy qz qw] --> 4*4 matrix

T = eye(4);
T(1:3,1:3) = quat_to_Rot(pose_quat_v(4:7));
T(1:3,4) = pose_quat_v(1:3);