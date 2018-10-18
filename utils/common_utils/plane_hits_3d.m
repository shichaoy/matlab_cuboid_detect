function pts_3d_world=plane_hits_3d(transToworld,invK,plane_sensor,pixels)
% compute pixel ray interesection with 3D plane in sensor frame. rays originates from camera center to pixel.
% transToworld: 4*4 camera pose.   invK: inverse of calibration matrix.   plane: 1*4  plane equation in sensor frame. 
% pixels  2*n; each column is a pt [x;y] x is horizontal,y is vertical   outputs: points 3*n in world frame

    pts_ray=invK*[pixels;ones(1,size(pixels,2))]; % 3*n
    
    pts_3d_sensor=ray_plane_interact(pts_ray,plane_sensor);
    pts_3d_homo_sensor=real_to_homo_coord(pts_3d_sensor);
    
    pts_3d_homo_world=transToworld*pts_3d_homo_sensor;   % compute world ground polygons.
    pts_3d_world=homo_to_real_coord(pts_3d_homo_world);
end
