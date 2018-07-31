function hit_pt = seg_hit_boundary(line_1, line_segment2)
% line_1  1*4  line_segment2 1*4  the output is float value
% compute the intersection of line_1 (from start to end) with line segments (not infinite line). if not found, return [-1 -1]
 %  the second line segments are either horizontal or vertical.   a simplified version of lineSegmentIntersect
   
    pt_start = line_1(1:2);
    pt_end = line_1(3:4);
    
    boundary_bgn = line_segment2(1:2);
    boundary_end = line_segment2(3:4);    
    
    direc = pt_end-pt_start;
    hit_pt=[-1 -1];
    % line equation is (p_u,p_v)+lambda*(delta_u,delta_v)  parameterized by lambda
    if (boundary_bgn(2)==boundary_end(2))  % if an horizontal edge
        lambd=(boundary_bgn(2)-pt_start(2))/direc(2);
        if (lambd>=0)  % along ray direction
            hit_pt_tmp = pt_start+lambd*direc;
            if (boundary_bgn(1)<=hit_pt_tmp(1)) && (hit_pt_tmp(1)<=boundary_end(1))  % inside the segments
                hit_pt = hit_pt_tmp;
                hit_pt(2)= boundary_bgn(2);  % floor operations might have un-expected things
            end
        end
    end
    
    if (boundary_bgn(1)==boundary_end(1))  % if an vertical edge
        lambd=(boundary_bgn(1)-pt_start(1))/direc(1);
        if (lambd>=0)  % along ray direction
            hit_pt_tmp=pt_start+lambd*direc;
            if (boundary_bgn(2)<=hit_pt_tmp(2)) && (hit_pt_tmp(2)<=boundary_end(2))  % inside the segments
                hit_pt = hit_pt_tmp;
                hit_pt(1)= boundary_bgn(1);  % floor operations might have un-expected things                
            end
        end
    end

end