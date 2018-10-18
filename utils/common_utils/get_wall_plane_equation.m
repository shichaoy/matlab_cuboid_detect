function plane_equation = get_wall_plane_equation(ground_seg3d_line_world)
% input: 1*6 a wall line segment in 3D ground. [x1 y1 z1  x2 y2 z2]  z1=z2=0    wall is vertical to ground
    
    partwall_normal_world=cross(ground_seg3d_line_world(1,1:3)-ground_seg3d_line_world(1,4:6),[0,0,1]); % [0,0,1] is world ground plane normal
    partwall_normal_world=partwall_normal_world/norm(partwall_normal_world);
    dist=-partwall_normal_world*ground_seg3d_line_world(1,1:3)';
    plane_equation=[partwall_normal_world dist];        % wall plane in world frame
    if (dist<0)        
        plane_equation=-plane_equation;   % make all the normal pointing inside the room. neamly, pointing to the camera
    end
