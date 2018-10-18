function total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles,vps_box_edge_pt_ids,box_corners_2d)
% evaluate box proposal by computing the angle difference of projected cuboid edge with VP aligned image edges.
% all_vp_bound_edge_angles: VP aligned actual image angles. 3*2  if not found, nan.     box_corners_2d: 2*8
% vps_box_edge_pt_ids: % six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP 

    total_angle_diff = 0;
    not_found_penalty = 30/180*pi*2;    % if not found any VP supported lines, give each box edge a constant cost (45 or 30 ? degree)
    for vp_id=1:size(vps_box_edge_pt_ids,1);
        vp_bound_angles=all_vp_bound_edge_angles(vp_id,:);                    
        vp_bound_angles=vp_bound_angles(~isnan((vp_bound_angles))); % row vector, will be 1*2  or 1*1 or 1*0
        found_matching_edge = false;
        if (size(vp_bound_angles,2)>0)  % exist valid edges
            if (vps_box_edge_pt_ids(vp_id,1)>0)
                for ee_id =1:2  % find cloeset from two boundary edges. we could also do left-left right-right compare. but pay close attention different vp locations                     
                    found_matching_edge = true;
                    two_box_corners = box_corners_2d(:, vps_box_edge_pt_ids(vp_id,2*ee_id-1:2*ee_id) ); % [ x1 x2;y1 y2 ]                        
                    box_edge_angle = normalize_to_pi( atan2(two_box_corners(2,2)-two_box_corners(2,1), two_box_corners(1,2)-two_box_corners(1,1)),true );  % [-pi/2 pi/2]                            
                    angle_diff_temp = abs(box_edge_angle - vp_bound_angles);   %vp_bound_angles is already in [-pi, pi/2]
                    % Edge distance could also be used. but might cause big error if edge is not detected or broken... and also if there is actually no edge, for example, chair might not have the ground contact edges.
                    angle_diff_temp = min(min( [angle_diff_temp;pi-angle_diff_temp],[],2));
                    total_angle_diff=total_angle_diff+angle_diff_temp;                
                end
            end
        end
        if (~found_matching_edge)
            angle_diff_temp = not_found_penalty; 
            total_angle_diff=total_angle_diff+angle_diff_temp;
        end
    end