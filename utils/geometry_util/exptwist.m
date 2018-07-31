function T = exptwist( twist )
% 6*1 or 1*6 twist rpyxyz

    omega = twist(1:3);
    
    if (length(twist)==6)
        upsilon = twist(4:6);
        if (size(upsilon,1)==1)
            upsilon = upsilon';
        end
    end
    
    theta = norm(omega);
    
    Omega = skew_matrix(omega);
    
    if (theta<0.00001)
        R = (eye(3) + Omega + Omega*Omega);
        V = R;
    else
        Omega2 = Omega*Omega;
        R = (eye(3)+ sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2);
        V = (eye(3)+(1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(theta^3)*Omega2);
    end
    
    if (length(twist)==6)
        T = eye(4);
        T(1:3,1:3) = R;
        T(1:3,4) = V*upsilon;
    else
        T = R;
    end
end