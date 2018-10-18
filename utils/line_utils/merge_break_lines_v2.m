function all_lines_merge = merge_break_lines_v2(all_lines,pre_merge_dist_thre,pre_merge_angle_thre)
% merge short edges into long. edges n*4  each edge should start from left to right!   this code mainly from c++ line_lbd
% thresholds for merging:  merge_dist_thre in pixel: endpoint distance of two line segments
%                          merge_angle_thre in degrees: angle between two lines.
                         
% -----   ----     two broken lines into one line

% ------------      close but offsetted lines not considered here, see merge_break_proj_lines.m
%    ------------


    all_lines_merge = all_lines;
    can_force_merge = 1;
    counter = 0;
    total_line_number = size(all_lines,1);
    if (size(all_lines,1)>0)
    while ( (can_force_merge==1) && (counter<500))
        counter=counter+1;
        can_force_merge=0;
        line_vector=all_lines_merge(:,3:4)-all_lines_merge(:,1:2);  % to compute the angles
        for seg1 = 1:total_line_number-1
            for seg2 = (seg1+1):total_line_number
                angle1= normalize_to_pi(atan2(line_vector(seg1,2),line_vector(seg1,1))/pi*180,false); % -90 ~ 90
                angle2= normalize_to_pi(atan2(line_vector(seg2,2),line_vector(seg2,1))/pi*180,false); % -90 ~ 90
                angle_diff = min(abs(angle1-angle2),180-abs(angle1-angle2));
                if (angle_diff < pre_merge_angle_thre)
                    dist_1ed_to_2 = norm(all_lines_merge(seg1,3:4)-all_lines_merge(seg2,1:2)); % one segment's end is close to another's begin
                    dist_2ed_to_1 = norm(all_lines_merge(seg2,3:4)-all_lines_merge(seg1,1:2));

                    if ( (dist_1ed_to_2 < pre_merge_dist_thre) || (dist_2ed_to_1 < pre_merge_dist_thre))
                        if all_lines_merge(seg1,1)<all_lines_merge(seg2,1)
                            merge_start = all_lines_merge(seg1,1:2);
                        else
                            merge_start = all_lines_merge(seg2,1:2);
                        end
                        if all_lines_merge(seg1,3)>all_lines_merge(seg2,3)
                            merge_end = all_lines_merge(seg1,3:4);
                        else
                            merge_end = all_lines_merge(seg2,3:4);
                        end
                        merged_angle = normalize_to_pi(atan2(merge_end(2)-merge_start(2),merge_end(1)-merge_start(1))/pi*180,false);
                        merge_angle_diff = min(abs(angle1-merged_angle),180-abs(angle1-merged_angle));
                        if (merge_angle_diff<pre_merge_angle_thre)                            
                            all_lines_merge(seg1,:) = [merge_start,merge_end];
                            
                            all_lines_merge(seg2,:) = all_lines_merge(total_line_number,:);
                            total_line_number=total_line_number-1;
%                             [seg1 seg2 total_line_number]
                            can_force_merge=1;
                            break
                        end
                    end
                end
            end
            if (can_force_merge==1)
                break;
            end
        end
    end
    end
    all_lines_merge=all_lines_merge(1:total_line_number,:);