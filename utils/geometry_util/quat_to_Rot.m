function rot = quat_to_Rot(quat)
% [qx qy qz qw] --> rotation matrix
	
	qx = quat(1);
	qy = quat(2);
	qz = quat(3);
    qw = quat(4);
    
    rot=[1 - 2*qy^2 - 2*qz^2,	2*qx*qy - 2*qz*qw,	2*qx*qz + 2*qy*qw;
         2*qx*qy + 2*qz*qw,	1 - 2*qx^2 - 2*qz^2,	2*qy*qz - 2*qx*qw;
         2*qx*qz - 2*qy*qw,	2*qy*qz + 2*qx*qw,	1 - 2*qx^2 - 2*qy^2];

end