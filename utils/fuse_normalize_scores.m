function [combined_error,all_delete_inds] = fuse_normalize_scores(dist_error, angle_error,weight_vp_angle,whether_normalize)
% fuse and normalize two kinds of cuboid proposal error, delete some bad proposals.
% output:  combined score: the score after delete some rows.   all_delete_inds: is the row indexes need to delete

    combined_error = [];
    all_delete_inds = [];
    if (size(dist_error,1)>0)
        
        % step 1. select top 2/3 of angle model and top 2/3 of distance model, find the union. then weighted fusion them. directly reject others.
        if (size(dist_error,1)>4)  % at least five to perform delete operations otherwise there are too few...
            [~, dist_sorted_inds] = sort(dist_error,'ascend');    % ascending order
            [angle_sorted_score, angle_sorted_inds] = sort(angle_error,'ascend');
            dist_delete_inds=dist_sorted_inds(round(size(dist_sorted_inds,1)/3*2):end);  % delete worst 1/3
            angle_delete_inds = [];
            delete_start_ind = round(size(angle_sorted_inds,1)/3*2);
            % since my angle error has setted maximum. maybe all cuboids have the maximum value. then ranking has no meaning.
            if ( angle_sorted_score(delete_start_ind)>angle_sorted_score(delete_start_ind-1) )
                angle_delete_inds=angle_sorted_inds(round(size(angle_sorted_inds,1)/3*2):end);
            end
            all_delete_inds = union(dist_delete_inds,angle_delete_inds);
            dist_error(all_delete_inds,:)=[];
            angle_error(all_delete_inds,:)=[];
        end

        % step 2:  fuse them.
        if (size(dist_error,1)>0)            
            dist_normalized = dist_error;
            angle_normalized = dist_error;
            
            if (whether_normalize)
                if (size(dist_error,1)>1)
                    dist_score_range = max(dist_error)-min(dist_error);
                    angle_score_range = max(angle_error)-min(angle_error);
                    dist_normalized = (dist_error-min(dist_error))/dist_score_range;  % actually max/min could be found on the fly.
                    if ((angle_score_range>0))   % angle error might all be same
                        angle_normalized = (angle_error-min(angle_error))/angle_score_range;        
                    end
                end
            end
            combined_error = (dist_normalized + weight_vp_angle*angle_normalized)/(1+weight_vp_angle);

        end
    end
    
    
    
    
