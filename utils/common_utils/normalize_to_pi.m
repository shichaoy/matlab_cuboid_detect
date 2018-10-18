function new_angle = normalize_to_pi(angle,whether_radian)
    %change angle from [-180,180] to [-90,90] through +-180    used to change line pointing to right.
    % if whether_radian: everything is changed to degree, instead of radian

    if (whether_radian)
        angle=angle/pi*180;
    end
        
    new_angle=angle;
    for ii=1:length(angle)
        if (angle(ii)>90)
            new_angle(ii) = angle(ii)-180;  % change to -90 ~90
        else if (angle(ii)<-90)
            new_angle(ii) = angle(ii)+180;
            else
                new_angle(ii) = angle(ii);
            end
        end
    end
    
    if (whether_radian)        
        new_angle=new_angle/180*pi;
    end
end
