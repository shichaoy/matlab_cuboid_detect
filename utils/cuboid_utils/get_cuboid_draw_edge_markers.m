function [edge_markers, line_marker_type] = get_cuboid_draw_edge_markers(box_config_type, final_universal_object)
% output: edge_markers  each row [ edge_start_pt_id, edge_end_pt_id,  edge_marker_type_id in line_marker_type ]
% box_config_type  [configuration_id, vp_1_on_left_or_right]      cuboid struct has this field.

    if (nargin==1)
        final_universal_object = true;
    end
    line_marker_type={'r','r--','g','g--','b','b--'};
    [visible_edge_pts, hidden_edge_pts] = get_object_edge_visibility(box_config_type, final_universal_object);
    edge_markers = [hidden_edge_pts; visible_edge_pts];  % draw hidden edge first
    
    if (final_universal_object)  % final saved cuboid struct
        if (box_config_type(1)==1)   
            if (box_config_type(2)==1)
                edge_markers = [edge_markers [4 2 6 3 1 5 5 5 3 1 3 1]' ];    % each row: edge_start_id,edge_end_id,edge_marker_type_id
            else
                edge_markers = [edge_markers [2 4 6 3 1 5 5 5 3 1 3 1]' ];
            end
        end
        if (box_config_type(1)==2)
            edge_markers = [edge_markers [2 4 2 6 6 3 5 5 3 1 3 1]' ];
        end
        if (box_config_type(1)==5)
            edge_markers = [edge_markers [2 4 2 2 4 2 6 6 3 5 5 3]' ];
        end
    else  % 2D box corners index only used in cuboids genetation process
        if (box_config_type(1)==1)
            edge_markers = [edge_markers [4 2 6 1 3 1 3 5 5 5 1 3]' ];
        else
            edge_markers = [edge_markers [4 2 6 6 2 1 3 1 3 5 5 3]' ];
        end
    end
end
    
    





