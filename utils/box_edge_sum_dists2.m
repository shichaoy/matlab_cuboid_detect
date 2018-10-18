function mean_dist = box_edge_sum_dists2(dist_map,box_corners_2d,edge_pt_ids, reweight_edge_distance)
% sample some points on the edge then sum up distance from dist_map
% input: visible_edge_pt_ids is n*2  each row stores an edge's two end point's index from box_corners_2d

% if weight_configs: for configuration 1, there are more visible edges compared to configuration2, so we need to re-weight config2
% [1 2;2 3;3 4;4 1;2 6;3 5;5 6]  reweight vertical edge id 5-6 by 2/3, horizontal edge id 7 by 1/2

% compared to box_edge_sum_dists(), longer edges sample more points! instead of fix 10

    sum_dist=0;
    if (nargin<4)
        reweight_edge_distance=false;
    end
    total_pt_num=0;
    for edge_id=1:size(edge_pt_ids,1)
        corner_tmp1=box_corners_2d(:, edge_pt_ids(edge_id,1));
        corner_tmp2=box_corners_2d(:, edge_pt_ids(edge_id,2));
        % sample points on this edge, every 10 pixel length, sample a point.
        edge_length = norm(corner_tmp2-corner_tmp1);
        sample_num = round(edge_length/10.0);
        total_pt_num = total_pt_num+sample_num;
        for sample_ind=1:sample_num
            sample_pt = round(sample_ind/sample_num*corner_tmp1+(1-sample_ind/sample_num)*corner_tmp2);
            dist1 = dist_map(sample_pt(2),sample_pt(1));
            if (reweight_edge_distance)  % two different configurations have different number of edges   true in config2
%                 if (5<=edge_id) && (edge_id<=7)
%                     dist1=dist1*2/3;
%                 end
%                 if (8<=edge_id)
%                     dist1=dist1*1/2;
%                 end
                
                if (5<=edge_id) && (edge_id<=6)
                    dist1=dist1*3/2;  
                end
                if (7==edge_id)
                    dist1=dist1*2;
                end
            end            
            sum_dist=sum_dist+dist1;            
        end
    end
    mean_dist = sum_dist/total_pt_num;
end