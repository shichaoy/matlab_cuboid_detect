function [intersections,frac]=ray_plane_interact(rays,plane)
% compute ray intersections with plane.
% rays is 3*n, each column is a ray vector endpoint staring from origin (0). plane is 1*4. all in sensor frame

    nume=-plane(4);
    denom = plane(1:3)*rays;
    frac=repmat(nume,1,size(denom,2))./denom;  % 1*n    
    intersections=repmat(frac,3,1).*rays;  %3*n
end
