function new_angles = smooth_jump_angles(raw_angles)
% remove the jumping angles from -pi to pi.   to make the raw angles smoothly change, to find the outmost or angle ray easily.
    
    new_angles = raw_angles;
    if (length(raw_angles)==0)
        return;
    end
        
    angle_base = raw_angles(1);  % choose a new base angle.   (assume that the all the angles lie in [-pi pi] around the base)
    for i=1:length(raw_angles)
        if ( (raw_angles(i)-angle_base)<-pi )
            new_angles(i) = raw_angles(i)+2*pi;
        elseif ( (raw_angles(i)-angle_base)>pi )
            new_angles(i) = raw_angles(i)-2*pi;
        end    
    end
end
