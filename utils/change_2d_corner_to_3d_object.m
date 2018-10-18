function sample_obj = change_2d_corner_to_3d_object(box_corners_2d_float,configs, ground_plane_sensor, transToWolrd, invK,projectionMatrix,sanity_check)
% unproject image corners to 3D ground, to form cuboid struct.  box_corners_2d_float is 2*8 image pixels.

    obj_gnd_pt_world_3d = plane_hits_3d(transToWolrd,invK,ground_plane_sensor,box_corners_2d_float(:,5:8));  % 3*n each column is a 3D point  floating point
%     figure();plot(obj_gnd_pt_world_3d(1,:),obj_gnd_pt_world_3d(2,:));axis equal
    
    length_half = norm(obj_gnd_pt_world_3d(1:2,1)-obj_gnd_pt_world_3d(1:2,4))/2;  % along object x direction   corner 5-8
    width_half = norm(obj_gnd_pt_world_3d(1:2,1)-obj_gnd_pt_world_3d(1:2,2))/2;   % along object y direction   corner 5-6
    
    partwall_plane_world = get_wall_plane_equation([obj_gnd_pt_world_3d(:,1)' obj_gnd_pt_world_3d(:,2)']);  % to compute height, need to unproject-hit-planes formed by 5-6 corner
    partwall_plane_sensor = partwall_plane_world*transToWolrd;  % wall plane in sensor frame
    obj_top_pt_world_3d = plane_hits_3d(transToWolrd,invK,partwall_plane_sensor,box_corners_2d_float(:,2));  % should match obj_gnd_pt_world_3d
    height_half = obj_top_pt_world_3d(3,1)/2;

    mean_obj_x = mean(obj_gnd_pt_world_3d(1,:)); mean_obj_y = mean(obj_gnd_pt_world_3d(2,:));

    vp_1_position = configs(2); yaw_esti = configs(3);
    sample_obj.pos = [mean_obj_x,mean_obj_y,height_half];  sample_obj.rotY = yaw_esti;
    sample_obj.scale = [length_half,width_half,height_half];
    sample_obj.box_config_type = configs(1:2);

    if (vp_1_position==1)  % vp1 on left, for all configurations
        cuboid_to_boxstructIds=[6 5 8 7 2 3 4 1];   % IMPORTANT!!! corner oder during generation is different from final universal cuboid struct
    end
    if (vp_1_position==2)  % vp1 on right, for all configurations
        cuboid_to_boxstructIds=[5 6 7 8 3 2 1 4];
    end
    
    if (configs(1)==5)
        vp_2_position = configs(2);
        if (vp_2_position==1)  % vp2 on left
            cuboid_to_boxstructIds=[7 8 5 6 1 4 3 2];
        end
        if (vp_2_position==2)  % vp2 on right
            cuboid_to_boxstructIds=[8 7 6 5 4 1 2 3];
        end
    end
    

    box_corners_2d_int = round(box_corners_2d_float);
    sample_obj.mymodel=1;
    sample_obj.box_corners_2d = box_corners_2d_int(:,cuboid_to_boxstructIds);   %  2*N  x;y        
    sample_obj.box_corners_3d_world = compute3D_BoxCorner(sample_obj);  % should match obj_gnd_pt_world_3d  this is my coordinate system, corner indexing is different

%     figure();
%     plot(obj_gnd_pt_world_3d(1,:),obj_gnd_pt_world_3d(2,:));axis equal  % the these two rectangles should match
%     plot(sample_obj.box_corners_3d_world(1,:),sample_obj.box_corners_3d_world(2,:),'b','Linewidth',2.5); xlabel('x');ylabel('y'); pause(0.5);    
    
    % check if the cuboid corners, position, indexing  are correct
    if (sanity_check)
        box_corners_2d_method_2 = round(projectToImage(sample_obj.box_corners_3d_world,projectionMatrix));   % should match the above box_corners_2d exactly!!
        if (any(sample_obj.box_corners_2d ~= box_corners_2d_method_2))  % if found non-equal number
            msg = 'Change to 3D box Sanity Check Failed !!!!';
            error(msg);
            box_corners_2d_method_2
            sample_obj.box_corners_2d
            disp(msg);
            sample_obj=[];
        end
    end