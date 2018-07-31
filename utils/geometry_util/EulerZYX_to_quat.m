function quat = EulerZYX_to_quat(euler_angle)
% intput: [roll, pitch, yaw] -->  [qx qy qz qw]
	roll = euler_angle(1);
	pitch = euler_angle(2);
	yaw = euler_angle(3);
	sy = sin(yaw*0.5);
	cy = cos(yaw*0.5);
	sp = sin(pitch*0.5);
	cp = cos(pitch*0.5);
	sr = sin(roll*0.5);
	cr = cos(roll*0.5);
	w = cr*cp*cy + sr*sp*sy;
	x = sr*cp*cy - cr*sp*sy;
	y = cr*sp*cy + sr*cp*sy;
	z = cr*cp*sy - sr*sp*cy;
    quat=[x y z w];
end
