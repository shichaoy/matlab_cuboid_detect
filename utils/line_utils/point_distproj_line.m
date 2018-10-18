function [dist,proj_percent]= point_distproj_line(line_begin_pt, line_end_pt, query_pt)
% comptue point distance to a infinite line and also projection position on the line segment   v w p   matching c++ popup
  
  length = norm(line_end_pt-line_begin_pt);
  if (length < 0.001) 
      dist=norm(query_pt-line_begin_pt);   % v == w case
      proj_percent=-1;
  else
      % Consider the line extending the segment, parameterized as v + t (w - v).
      % We find projection of point p onto the line. 
      % It falls where t = [(p-v) . (w-v)] / |w-v|^2
      t = dot( (query_pt-line_begin_pt),(line_end_pt-line_begin_pt) )/length/length;
      projection = line_begin_pt + t * (line_end_pt - line_begin_pt);
      dist = norm(query_pt-projection);
      if (t>1)  % cut into [0 1]
        t=1;
      end
      if (t<0)
        t=0;       
      end
      proj_percent=t;
  end