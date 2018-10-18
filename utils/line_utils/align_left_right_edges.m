function all_lines_raw = align_left_right_edges(all_lines_raw)
% make edge always start from left to right x1 < x2   lines: n*4  [x1 y1 x2 y2]

    for line_ind=1:size(all_lines_raw,1)        
        if (all_lines_raw(line_ind,3) < all_lines_raw(line_ind,1))  % each edge starts from left to right
                temp=all_lines_raw(line_ind,3:4);
                all_lines_raw(line_ind,3:4)=all_lines_raw(line_ind,1:2);
                all_lines_raw(line_ind,1:2)=temp;
        end
    end
end