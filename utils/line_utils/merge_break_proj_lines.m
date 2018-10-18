function all_lines_merge = merge_break_proj_lines(all_lines,pre_merge_dist_thre,pre_merge_angle_thre,pre_proj_cover_thre,pre_proj_dist_thre)
% more complicated! merge short edges into long. edges n*4  each edge should start from left to right!  mainly from c++ pop_up
% thresholds for merging:  merge_dist_thre in pixel: endpoint distance of two line segments
%                          merge_angle_thre in degrees: angle between two lines.
%                          proj_cover_thre: projection threshold.  if overlap too large, delete one.  otherwise consider merging
%                          proj_dist_thre: one line endpoint distance to another (infinite) line

% ----- ----    1end--2begin is close    merge broken lines

% ------------  overlapping is large, delete one, remain one     duplicate lines
%    ----

% ------       ----- -----     overlapping is small,  endpoint is close, then merge.  broken duplicate lines   if far, don't merge
%      -----



    all_lines_merge = all_lines;
    can_force_merge = 1;
    counter = 0;
    if (nargin<5)
        pre_proj_dist_thre=15;  % 15 pixels distance between almost parallel lines
    end
    if (nargin<4)
        pre_proj_cover_thre = 0.6;  % if have very large overlap, delete one of them! else merge them
    end
    if (size(all_lines,1)>0)
    while ( (can_force_merge==1) && (counter<500))
        counter=counter+1;
        can_force_merge=0;
        line_vector = all_lines_merge(:,3:4)-all_lines_merge(:,1:2);  % to compute the angles
        for seg1 = 1:size(all_lines_merge,1)
            angle1= normalize_to_pi(atan2(line_vector(seg1,2),line_vector(seg1,1))/pi*180,false); % -90 ~ 90
            for seg2 = (seg1+1):size( all_lines_merge,1)                
                angle2= normalize_to_pi(atan2(line_vector(seg2,2),line_vector(seg2,1))/pi*180,false); % -90 ~ 90
                angle_diff = min(abs(angle1-angle2),180-abs(angle1-angle2));
                if (angle_diff < pre_merge_angle_thre)
                    [dist_1_bg_to_2,proj_1_bg_to_2]= point_distproj_line(all_lines_merge(seg2,1:2), all_lines_merge(seg2,3:4), all_lines_merge(seg1,1:2));
                    [dist_1_ed_to_2,proj_1_ed_to_2]= point_distproj_line(all_lines_merge(seg2,1:2), all_lines_merge(seg2,3:4), all_lines_merge(seg1,3:4));
                    [dist_2_bg_to_1,proj_2_bg_to_1]= point_distproj_line(all_lines_merge(seg1,1:2), all_lines_merge(seg1,3:4), all_lines_merge(seg2,1:2));
                    [dist_2_ed_to_1,proj_2_ed_to_1]= point_distproj_line(all_lines_merge(seg1,1:2), all_lines_merge(seg1,3:4), all_lines_merge(seg2,3:4));
                    if ( (dist_1_bg_to_2<pre_proj_dist_thre) && (dist_1_ed_to_2<pre_proj_dist_thre) && ...
                        (dist_2_bg_to_1<pre_proj_dist_thre) && (dist_2_ed_to_1<pre_proj_dist_thre))    % two lines are close in projection distance, then consider merge
                        covering_1_to_2 = abs(proj_1_bg_to_2-proj_1_ed_to_2);
                        covering_2_to_1 = abs(proj_2_bg_to_1-proj_2_ed_to_1);                        
                        if ((covering_1_to_2>pre_proj_cover_thre) || (covering_2_to_1>pre_proj_cover_thre))  %at least one line is being overlapped larged  delete one which is being overlapped more
                            if (covering_1_to_2>covering_2_to_1)
                                to_delete_ind=seg2;
                            else
                                to_delete_ind=seg1;
                            end
                            all_lines_merge(to_delete_ind,:) = [];
                            can_force_merge=1;
                            break;
                        else   % two lines have small overlap with each other, check their end point distance    if close, then merge    merge broken lines
                            dist_1end_to_2bg = norm(all_lines_merge(seg1,3:4)-all_lines_merge(seg2,1:2));
                            dist_2end_to_1bg = norm(all_lines_merge(seg2,3:4)-all_lines_merge(seg1,1:2));
                            if ( (dist_1end_to_2bg < pre_merge_dist_thre) || (dist_2end_to_1bg < pre_merge_dist_thre))
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
                                    all_lines_merge(seg2,:) = [];
                                    can_force_merge=1;
                                    break;
                                end
                            end
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