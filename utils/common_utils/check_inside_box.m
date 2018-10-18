function whether_inside = check_inside_box( pt, box_left_top, box_right_bottom)
% check whether point lies inside rectangle box   input are three 2D points [x y]

    whether_inside = box_left_top(1)<=pt(1) && pt(1)<=box_right_bottom(1) && box_left_top(2)<=pt(2) && pt(2)<=box_right_bottom(2);
