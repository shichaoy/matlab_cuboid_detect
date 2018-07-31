function [euler] = quat_to_EulerZYX(quat)
	
	qx = quat(1);
	qy = quat(2);
	qz = quat(3);
    qw = quat(4);
	
	roll = atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy));
	pitch = asin(2*(qw*qy-qz*qx));
	yaw = atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz));

    euler = [roll, pitch, yaw];
end