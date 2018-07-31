function [ euler ] = Rot_to_EulerZYX( R )
% output is [roll pitch yaw]
pitch = asin(-R(3,1));

if (abs(pitch - pi/2) < 1.0e-3) 
    roll = 0.0;
    yaw = atan2(R(2,3) - R(1,2), R(1,3) + R(2,2)) + roll;
elseif (abs(pitch + pi/2) < 1.0e-3) 
    roll = 0.0;
    yaw = atan2(R(2,3) - R(1,2), R(1,3) + R(2,2)) - roll;
else 
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
end

euler = [roll;pitch;yaw];


end



% euler(2) = asin(-R(3,1));
% 
% if (abs(euler(2) - pi/2) < 1.0e-3) 
%     euler(1) = 0.0;
%     euler(3) = atan2(R(2,3) - R(1,2), R(1,3) + R(2,2)) + euler(1);
% elseif (abs(euler(2) + pi/2) < 1.0e-3) 
%     euler(1) = 0.0;
%     euler(3) = atan2(R(2,3) - R(1,2), R(1,3) + R(2,2)) - euler(1);
% else 
%     euler(1) = atan2(R(3,2), R(3,3));
%     euler(3) = atan2(R(2,1), R(1,1));
% end
