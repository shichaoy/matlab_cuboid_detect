function all_lines_long = remove_short_lines(all_lines,edge_length_threshold,angle_threshold)
% lines n*4   remove short and near vertical lines
% angle_threshold: angle difference to 90'  too vertical lines are not assumed to be wall edges.

    if (size(all_lines,1)>0)
        line_vector=all_lines(:,3:4)-all_lines(:,1:2);        
                
        all_line_angles = normalize_to_pi(atan2(line_vector(:,2),line_vector(:,1))/pi*180,false); % -90 ~ 90  degree
        if (nargin<=2)
            all_lines_long=all_lines( sum(line_vector.*line_vector,2)>edge_length_threshold*edge_length_threshold,:);            
        else            
            long_lines_ind = sum(line_vector.*line_vector,2)>edge_length_threshold*edge_length_threshold;            
            angle_diff = min([abs(all_line_angles-90),180-abs(all_line_angles-90)],[],2);  % angle diff wrt vertical line
            non_vertical_ind = angle_diff>angle_threshold;
            all_lines_long=all_lines((long_lines_ind.*non_vertical_ind)==1,:);
        end
    else
        all_lines_long=[];
    end
end
