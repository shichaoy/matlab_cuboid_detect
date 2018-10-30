function   all_vp_bound_edge_angles = VP_support_edge_infos(VPs,edge_mid_pts,edge_angles,vp_support_angle_thres)
% VPs 3*2  three VP   edge_mid_pts: n*2  edge_angles: n*1 vp_support_angle_thres 1*2
% output: 3*2  each row is a VP's two boundary supported edges' angle.  if not found, nan for that entry
% [edge_angles all_vp_bound_edge_angles] are all [-pi, pi]

    if (size(edge_mid_pts,1)>0)    
        vp_1 = VPs(1,:);vp_2 = VPs(2,:);vp_3 = VPs(3,:);
        vp12_edge_angle_thre=vp_support_angle_thres(1);
        vp3_edge_angle_thre=vp_support_angle_thres(2);

        % first find edges that are supported by (pass through) VP
        vp1_edge_midpt_angle_raw=atan2( edge_mid_pts(:,2)-vp_1(2), edge_mid_pts(:,1)-vp_1(1));
        vp1_edge_midpt_angle_norm = normalize_to_pi(vp1_edge_midpt_angle_raw,true);  % [-pi/2 pi/2]   angle of [vp, edge middle pt]
        angle_diff_1 = abs(edge_angles-vp1_edge_midpt_angle_norm);  % [0 pi]
        angle_diff_1 = min([angle_diff_1,pi-angle_diff_1],[],2);
        vp1_inlier_edge_id = find(angle_diff_1 < vp12_edge_angle_thre/180*pi); 
        
        vp2_edge_midpt_angle_raw=atan2( edge_mid_pts(:,2)-vp_2(2), edge_mid_pts(:,1)-vp_2(1));
        vp2_edge_midpt_angle_norm = normalize_to_pi(vp2_edge_midpt_angle_raw,true);
        angle_diff_2 = abs(edge_angles-vp2_edge_midpt_angle_norm);
        angle_diff_2 = min([angle_diff_2,pi-angle_diff_2],[],2);
        vp2_inlier_edge_id = find(angle_diff_2 < vp12_edge_angle_thre/180*pi);
        
        if ~isnan(vp_3(1))  % nan because vp_3 is vertically infinity
            vp3_edge_midpt_angle_raw = atan2(edge_mid_pts(:,2)-vp_3(2), edge_mid_pts(:,1)-vp_3(1));            
        else
            vp3_edge_midpt_angle_raw = pi/2*ones(size(edge_angles));
        end
        vp3_edge_midpt_angle_norm = normalize_to_pi(vp3_edge_midpt_angle_raw,true);
        
        angle_diff_3 = abs(edge_angles-vp3_edge_midpt_angle_norm);
        angle_diff_3 = min([angle_diff_3,pi-angle_diff_3],[],2);
        vp3_inlier_edge_id = find(angle_diff_3 < vp3_edge_angle_thre/180*pi);


        % find VP supported Boundary Edges.    % TODO min/max actually needs to change if want to    vp1 in left/right is different   min/max reversed
        vp1_edge_midpt_angle_raw_inlier = vp1_edge_midpt_angle_raw(vp1_inlier_edge_id,:);
        vp1_edge_midpt_angle_raw_inlier_shift = smooth_jump_angles(vp1_edge_midpt_angle_raw_inlier);
        [~,vp1_low_edge_id] = max(vp1_edge_midpt_angle_raw_inlier_shift);  %  image coordinate goes down, yaw clockwise large.  % NOTE if vp1 on the right. should switch min/max....
        vp1_low_edge_angle = edge_angles(vp1_inlier_edge_id(vp1_low_edge_id));    % it will be 0*1 matrix if not found inlier edges.
        [~,vp1_top_edge_id] = min(vp1_edge_midpt_angle_raw_inlier_shift);
        vp1_top_edge_angle = edge_angles(vp1_inlier_edge_id(vp1_top_edge_id));
        
        vp2_edge_midpt_angle_raw_inlier = vp2_edge_midpt_angle_raw(vp2_inlier_edge_id,:);
        vp2_edge_midpt_angle_raw_inlier_shift = smooth_jump_angles(vp2_edge_midpt_angle_raw_inlier);
        [~,vp2_low_edge_id] = min(vp2_edge_midpt_angle_raw_inlier_shift);
        vp2_low_edge_angle = edge_angles(vp2_inlier_edge_id(vp2_low_edge_id));    
        [~,vp2_top_edge_id] = max(vp2_edge_midpt_angle_raw_inlier_shift);
        vp2_top_edge_angle = edge_angles(vp2_inlier_edge_id(vp2_top_edge_id));
        
        vp3_edge_midpt_angle_raw_inlier = vp3_edge_midpt_angle_raw(vp3_inlier_edge_id,:);
        vp3_edge_midpt_angle_raw_inlier_shift = smooth_jump_angles(vp3_edge_midpt_angle_raw_inlier);
        [~,vp3_left_edge_id] = min(vp3_edge_midpt_angle_raw_inlier_shift);
        vp3_left_edge_angle = edge_angles(vp3_inlier_edge_id(vp3_left_edge_id));    
        [~,vp3_right_edge_id] = max(vp3_edge_midpt_angle_raw_inlier_shift);
        vp3_right_edge_angle = edge_angles(vp3_inlier_edge_id(vp3_right_edge_id));
    else
        vp1_low_edge_angle=[];vp1_top_edge_angle=[];
        vp2_low_edge_angle=[];vp2_top_edge_angle=[];
        vp3_left_edge_angle=[];vp3_right_edge_angle=[];
    end

    if size(vp1_low_edge_angle,1)==0   % if no inlier edges. possibly because of too strict edge support threshold
        vp1_low_edge_angle=nan;
    end
    if size(vp1_top_edge_angle,1)==0
        vp1_top_edge_angle=nan;
    end
    if size(vp2_low_edge_angle,1)==0
        vp2_low_edge_angle=nan;
    end
    if size(vp2_top_edge_angle,1)==0
        vp2_top_edge_angle=nan;
    end
    if size(vp3_left_edge_angle,1)==0
        vp3_left_edge_angle=nan;
    end
    if size(vp3_right_edge_angle,1)==0
        vp3_right_edge_angle=nan;
    end
    
    all_vp_bound_edge_angles = [vp1_low_edge_angle vp1_top_edge_angle;
                               vp2_low_edge_angle vp2_top_edge_angle;
                               vp3_left_edge_angle vp3_right_edge_angle];
    
