function x=real_to_homo_coord(pts)
    %d*n --> (d+1)*n
    x=[pts; ones(1,size(pts,2))];    
        
