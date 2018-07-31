function R = EulerZYX_to_Rot(euler_angle)

roll = euler_angle(1);
pitch = euler_angle(2);
yaw = euler_angle(3);

cp = cos(pitch);
sp = sin(pitch);
sr = sin(roll);
cr = cos(roll);
sy = sin(yaw);
cy = cos(yaw);

R = [cp * cy    (sr * sp * cy) - (cr * sy)      (cr * sp * cy) + (sr * sy);
     cp * sy    (sr * sp * sy) + (cr * cy)      (cr * sp * sy) - (sr * cy);
     -sp        sr * cp                         cr * cp];
