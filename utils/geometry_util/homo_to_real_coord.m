function x=homo_to_real_coord(pts_homo)
    % d*n  --> (d-1)*n
    x=pts_homo./repmat(pts_homo(end,:),size(pts_homo,1),1);
    x=x(1:end-1,:);
    