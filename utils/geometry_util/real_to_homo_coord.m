function x=real_to_homo_coord(pts)
% real coords to homogeneous coords  d*n --> (d+1)*n

    x=[pts; ones(1,size(pts,2))];    
        
