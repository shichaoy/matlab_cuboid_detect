function [visible_edge_pts,hidden_edge_pts] = get_object_edge_visibility(box_config_type, final_universal_object)
% Output: n*2  each row is a edge's start and end pt id. 
% box_config_type  [configuration_id, vp_1_on_left_or_right]      cuboid struct has this field.

    if (nargin==1)
        final_universal_object = true;
    end
    
    if (final_universal_object)   % final saved cuboid struct
        if (box_config_type(1)==1)        % look at get_cuboid_face_ids to know the faces and pt id using my old box format
            if (box_config_type(2)==1)
                visible_edge_pts=[1 2;2 3;2 6;1 5;3 7;5 6;6 7;7 8;8 5];
                hidden_edge_pts=[3 4;4 1;4 8];
            else
                visible_edge_pts=[1 2;1 4;2 6;1 5;4 8;5 6;6 7;7 8;8 5];
                hidden_edge_pts=[2 3;3 4;3 7];
            end
        end
        if (box_config_type(1)==2)
            visible_edge_pts=[1 2;2 6;1 5;5 6;6 7;7 8;8 5];
            hidden_edge_pts=[2 3;3 4;4 1;3 7;4 8];
        end
        if (box_config_type(1)==5)                
            visible_edge_pts=[1 2;2 6;1 5;5 6];
            hidden_edge_pts=[6 7;7 8;8 5;2 3;3 4;4 1;3 7;4 8];
        end
    else   % 2D box corners index only used in cuboids genetation process
        if (box_config_type(1)==1)
            visible_edge_pts = [1 2;2 3;3 4;4 1;2 6;3 5;4 8;5 8;5 6];
            hidden_edge_pts = [7 8;7 6;7 1];
        else
            visible_edge_pts = [1 2;2 3;3 4;4 1;2 6;3 5;5 6];
            hidden_edge_pts = [7 8;7 6;7 1;8 4;8 5];
        end        
    end

    