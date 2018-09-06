clear;
close all;

load('data/frame_full_infos.mat');   % load frame pose information
load('data/all_select_2d_bboxes.mat');  % load 2d bbox


%% set important hyper paramaters
% default parameters.
get_relative_measures = true;         % true if set camera x=y=yaw=0, only provide height, roll, pitch,  for object SLAM cuboid coordinates should be in local frame
whether_sample_bbox_height = false;   % sample object height as raw detection might not be accurate. top fixed. sample down. affect cuboid distance
whether_sample_roll_pitch = false;    % sample camera roll pitch in case don't have/want ground truth camera pose
whether_sample_cam_height = false;    % sample camera height. top fixed, sample down.

consider_config_1 = true;  % see three faces   false  true
consider_config_2 = true;  % see two faces
consider_config_3 = false;  % see one face   not perfect, mainly for kitti vertical obj.


%% start processing
frame_index = 0;
disp(['Processing image ',num2str(frame_index)]);

frame_calib_mat = frame_full_infos(frame_index+1); % matlab ind start from 1

whether_plot_detail_img = false; % true false

rgb_img = imread([sprintf('data/%04d_rgb_raw.jpg',frame_index)]);
if length(size(rgb_img))==2
    rgb_img = cat(3, rgb_img, rgb_img, rgb_img);
end

img_width=size(rgb_img,2);  img_height=size(rgb_img,1);
if(whether_plot_detail_img)
    figure(1); imshow(rgb_img);title('Raw Image');hold on;
end


% read selected yolo 2d object bounding boxes
obj_bbox_coors = all_select_2d_bboxes{frame_index+1};   % the object index here start from 1
if (exist('all_select_2dboxes_classes'))
    obj_bbox_class = all_select_2dboxes_classes{frame_index+1};  % class name no use.
else
    obj_bbox_class = cell(size(obj_bbox_coors,1),1);  % class name no use.
end
object_colors={'red','green','blue','cyan','magenta','yellow','red','green','blue','cyan'};
if (size(obj_bbox_coors,1)==0)    
    fprintf('Skip, Not found valid 2D object of frame %d \n',frame_index);
end

%% camera information
euler_angle = Rot_to_EulerZYX(frame_calib_mat.Rot);  % .Rot is camera rotation wrt. ground.
init_roll = euler_angle(1);
init_pitch = euler_angle(2);
if (get_relative_measures)  % actually we can do this later separately
    frame_calib_mat.position(1:2) = 0;  % set x y=0
    euler_angle(3)=0;      % set yaw = 0
    frame_calib_mat.Rot = EulerZYX_to_Rot(euler_angle);
end
camera_yaw = euler_angle(3);
transToWolrd = eye(4);   %  transToWolrd*P_camera = P_world   transform a point in camera frame to world frame.
transToWolrd(1:3,1:3) = frame_calib_mat.Rot;
transToWolrd(3,4)= frame_calib_mat.camera_height;  % use estimated camera height, or 1 if not.
rotationToWorld = transToWolrd(1:3,1:3);
if (isfield(frame_calib_mat,'position'))  % set x y position
    transToWolrd(1:2,4)=frame_calib_mat.position(1:2);
end
invT = inv(transToWolrd);
invR = inv(rotationToWorld);
Kalib = frame_calib_mat.K;  % internal calibration
invK = inv(Kalib);
projectionMatrix = Kalib*invT(1:3,:);  % project world coordinate to image

ground_plane_world = [0,0,1,0];  % 1*4   % in my pop-up code, I use [0 0 -1 0]. here I want the normal pointing innerwards, towards the camera to match surface normal prediction
ground_plane_sensor=ground_plane_world*transToWolrd;


% some parameters
shorted_edge_thre = 20;  % if box edge are too short. box might be too thin. most possibly wrong.        


for object_id = 1:size(obj_bbox_coors,1)
    left_x_raw = obj_bbox_coors(object_id,1); top_y_raw = obj_bbox_coors(object_id,2); obj_width_raw=obj_bbox_coors(object_id,3); obj_height_raw=obj_bbox_coors(object_id,4);
    right_x_raw=left_x_raw+obj_width_raw; down_y_raw = top_y_raw + obj_height_raw;    

    down_expand_sample_all = [0];  % whether sample 2d bbox height
    sample_cam_rolls_all = [init_roll];  % whether sample camera roll/pitch
    sample_cam_pitches_all = [init_pitch];
    sample_cam_heights_all = [frame_calib_mat.camera_height];  % whether sample camera height

    all_box_corners_2ds = [];
    for down_expand_sample = down_expand_sample_all
        obj_height_expan = obj_height_raw + down_expand_sample;
        down_y_expan = top_y_raw + obj_height_expan;    obj_diaglength_expan = sqrt(obj_width_raw^2+obj_height_expan^2);
        
        whether_plot_detail_img=false;  % true false
        if (whether_plot_detail_img)
            figure(20);imshow(rgb_img);hold on;
            hh=rectangle('Position',[left_x_raw top_y_raw,obj_width_raw,obj_height_expan],'EdgeColor',object_colors{object_id},'LineWidth',2); hh.LineStyle='--';
            pause(0.5);
        end
        
        % sample points on the top edges, if edge is too large, give more samples. give at least 10 samples for all edges.
        top_sample_resolution=round(min(20,obj_width_raw/10 )); %  at most 20 pixels per sample
        sample_top_pts = left_x_raw+5:top_sample_resolution:right_x_raw-5;sample_top_pts=[sample_top_pts;top_y_raw*ones(1,size(sample_top_pts,2))];
        
        % expand some small margin for distance map
        distmap_expand_wid = min(max(min(20, obj_width_raw-100),10),max(min(20, obj_height_expan-100),10)); 
        left_x_expan_distmap = max(1,left_x_raw-distmap_expand_wid);right_x_expan_distmap = min(img_width,right_x_raw+distmap_expand_wid);
        top_y_expan_distmap = max(1,top_y_raw-distmap_expand_wid);down_y_expan_distmap = min(img_height,down_y_expan+distmap_expand_wid);

        
        yaw_init = camera_yaw-90/180*pi;  % object yaw initialized to face camera

        all_box_corners_2d_conf1_raw =[];
        all_box_corners_2d_conf2_raw =[];
        all_box_corners_2d_conf3_raw =[];        
        for sample_cam_roll = sample_cam_rolls_all
        for sample_cam_pitch = sample_cam_pitches_all
        for sample_cam_height = sample_cam_heights_all
            transToWolrd(1:3,1:3) = EulerZYX_to_Rot([sample_cam_roll sample_cam_pitch camera_yaw]);
            transToWolrd(3,4) = sample_cam_height;
            rotationToWorld = transToWolrd(1:3,1:3);
            invT = inv(transToWolrd);
            invR = inv(rotationToWorld);
            projectionMatrix = Kalib*invT(1:3,:);
            ground_plane_sensor = ground_plane_world*transToWolrd;

            if(whether_plot_detail_img)
                % compute initial vanishing points
                [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_init); % for object x y z  axis                    
                figure(25);clf;title('detected Vps');hold on;
                imagesc(rgb_img);hold on;
                plot(vp_1(1),vp_1(2),'r*','MarkerSize',20);
                plot(vp_2(1),vp_2(2),'g*','MarkerSize',20);
                plot(vp_3(1),vp_3(2),'b*','MarkerSize',20);hold off;
                axis ij;axis equal;pause(0.5);
            end
            
            whether_plot_detail_img=true; plot_cube_generate_detail=true; print_details=false;  % false  true    
            obj_yaw_samples = yaw_init-45/180*pi:6/180*pi:yaw_init+45/180*pi;  % search an object raw range of 90

            %% configuration one  can see three faces
            if (consider_config_1)
            all_box_corners_2d_conf1 =[];            
            figure_id_1 = 60;            
            for yaw_sample_id = 1:size(obj_yaw_samples,2)
                yaw_esti = obj_yaw_samples(yaw_sample_id);
                [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                if (isnan(vp_3(1))) vp_3=[nan nan]; end;
                
                for sample_top_pt_id=1:size(sample_top_pts,2)
                    sample_cond = [yaw_sample_id  sample_top_pt_id];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);imshow(rgb_img);title('Config 1  Detected 2D object --> 3D');hold on;
                        rectangle('Position',[left_x_raw,top_y_raw,right_x_raw-left_x_raw,down_y_expan-top_y_raw],'EdgeColor','c','LineWidth',1); 
%                         text(obj_bbox_coors(object_id,1)+5,obj_bbox_coors(object_id,2)-10, sprintf('%s---%.2f',obj_bbox_class{1},obj_bbox_coors(object_id,5)))            
                    end

                    corner_1_top = sample_top_pts(:,sample_top_pt_id)';
                    config_1_good = true;

                    vp_1_position = 0;  % 0 initial as fail,  1  on left   2 on right      configuration 1 or 4. may be later if I change the yaw. it will be the same        
                    corner_2_top = seg_hit_boundary([vp_1 corner_1_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);        
                    if (corner_2_top(1)==-1)  % vp1-corner1 doesn't hit the right boundary. check whether hit left
                        corner_2_top = seg_hit_boundary([vp_1 corner_1_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                        if (corner_2_top(1)~=-1) % vp1-corner1 hit the left boundary   vp1 on the right
                            vp_1_position = 2;
                        end
                    else    % vp1-corner1 hit the right boundary   vp1 on the left
                        vp_1_position = 1;
                    end
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);plot([corner_1_top(1) corner_2_top(1)],[corner_1_top(2) corner_2_top(2)],'r','Linewidth',2.5);        
                    end        
                    config_1_good = (vp_1_position>0);   % at least find one intersection for corner_2
                    if (~config_1_good)
                        if (print_details) disp('Configuration one fails at corner 2, outside segment'); end;  continue;
                    end
                    if (norm(corner_1_top-corner_2_top)<shorted_edge_thre)
                        if (print_details) disp('Configuration one fails at edge 1-2, too short');  end;  continue;
                    end

                    if (vp_1_position==1)   % then vp2 hit the left boundary
                        corner_4_top = seg_hit_boundary([vp_2 corner_1_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                        if(whether_plot_detail_img && plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                        end
                    else   % or, then vp2 hit the right boundary
                        corner_4_top = seg_hit_boundary([vp_2 corner_1_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                        if(whether_plot_detail_img && plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                        end
                    end

                    if (corner_4_top(1)==-1)
                        config_1_good = false; if (print_details)  disp('Configuration one fails at corner 4, outside segment'); end;  continue;
                    end
                    if (norm(corner_1_top-corner_4_top)<shorted_edge_thre)
                        if (print_details) disp('Configuration one fails at edge 1-4, too short'); end;  continue;
                    end

                    % compute the last point in the top face
                    intersect_result=lineSegmentIntersect([vp_2  corner_2_top],[vp_1 corner_4_top],true);
                    corner_3_top=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);plot([corner_3_top(1) corner_2_top(1)],[corner_3_top(2) corner_2_top(2)],'g','Linewidth',2.5);
                        figure(figure_id_1);plot([corner_3_top(1) corner_4_top(1)],[corner_3_top(2) corner_4_top(2)],'r','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_3_top, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))    % check inside boundary. otherwise edge visibility might be wrong           
                        config_1_good=false;  if (print_details) disp('Configuration one fails at corner 3, outside box'); end;  continue;
                    end
                    if ( (norm(corner_3_top-corner_4_top)<shorted_edge_thre) || ((norm(corner_3_top-corner_2_top)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration one fails at edge 3-4/3-2, too short'); end;  continue;
                    end
                    % compute first bottom points
                    corner_5_down = seg_hit_boundary([vp_3 corner_3_top],[left_x_raw down_y_expan right_x_raw down_y_expan]); % vp_3 is vertial in kitti
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);plot([corner_3_top(1) corner_5_down(1)],[corner_3_top(2) corner_5_down(2)],'b','Linewidth',2.5);
                    end
                    if (corner_5_down(1)==-1)
                       config_1_good = false; if (print_details)  disp('Configuration one fails at corner 5, outside segment'); end; continue;
                    end
                    if (norm(corner_3_top-corner_5_down)<shorted_edge_thre)
                        if (print_details) disp('Configuration one fails at edge 3-5, too short'); end;  continue;
                    end
                    % don't need to check bottom points, it actually includes some other configurations...
                    intersect_result=lineSegmentIntersect([vp_2 corner_5_down],[vp_3 corner_2_top],true);
                    corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b','Linewidth',2.5);
                        figure(figure_id_1);plot([corner_6_down(1) corner_5_down(1)],[corner_6_down(2) corner_5_down(2)],'g','Linewidth',2.5);                
                    end
                    if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
%                     if (~check_inside_box( corner_6_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))
                        config_1_good=false; if (print_details) disp('Configuration one fails at corner 6, outside image'); end; continue;
                    end
                    if ( (norm(corner_6_down-corner_2_top)<shorted_edge_thre) || ((norm(corner_6_down-corner_5_down)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration one fails at edge 6-5/6-2, too short'); end; continue;
                    end
                    intersect_result=lineSegmentIntersect([vp_1 corner_6_down],[vp_3 corner_1_top],true);
                    corner_7_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_1);plot([corner_7_down(1) corner_1_top(1)],[corner_7_down(2) corner_1_top(2)],'b--','Linewidth',2.5);                
                        figure(figure_id_1);plot([corner_7_down(1) corner_6_down(1)],[corner_7_down(2) corner_6_down(2)],'r--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_7_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
%                     if (~check_inside_box( corner_7_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))  % 7 must line inside object. hidden point                              
                        config_1_good=false; if (print_details) disp('Configuration one fails at corner 7, outside image'); end; continue;
                    end
                    if ( (norm(corner_7_down-corner_1_top)<shorted_edge_thre) || ((norm(corner_7_down-corner_6_down)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration one fails at edge 7-1/7-6, too short'); end; continue;
                    end            
                    intersect_result=lineSegmentIntersect([vp_1 corner_5_down],[vp_2 corner_7_down],true);  % could also use vp3_corner4, but sometimes nearly parallel
                    corner_8_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)                
                        figure(figure_id_1);plot([corner_8_down(1) corner_4_top(1)],[corner_8_down(2) corner_4_top(2)],'b','Linewidth',2.5);
                        figure(figure_id_1);plot([corner_8_down(1) corner_5_down(1)],[corner_8_down(2) corner_5_down(2)],'r','Linewidth',2.5);
                        figure(figure_id_1);plot([corner_8_down(1) corner_7_down(1)],[corner_8_down(2) corner_7_down(2)],'g--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_8_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
%                     if (~check_inside_box( corner_8_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))                                
                        config_1_good=false; if (print_details) disp('Configuration one fails at corner 8, outside image');end;  continue;
                    end
                    if ( (norm(corner_8_down-corner_4_top)<shorted_edge_thre) || (norm(corner_8_down-corner_5_down)<shorted_edge_thre) || (norm(corner_8_down-corner_7_down)<shorted_edge_thre) )
                        if (print_details) disp('Configuration one fails at edge 8-4/8-5/8-7, too short'); end; continue;
                    end

                    box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                    box_corners_2d_int = round(box_corners_2d_float);
                    all_box_corners_2d_conf1 = [all_box_corners_2d_conf1; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]

                    if (whether_plot_detail_img && plot_cube_generate_detail)  % only plot valid boxes
                        pause(0.5);
                    end
                end
            end

            all_box_corners_2d_conf1_raw = [all_box_corners_2d_conf1_raw; all_box_corners_2d_conf1];            
            end

            %% configuration two  can see two faces
            if (consider_config_2)
            all_box_corners_2d_conf2 =[];   
            figure_id_3 = 70;
            % the top four points are generatedly differently, the bottom four are same, except with different visibility
            for yaw_esti = obj_yaw_samples    
                [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                if (isnan(vp_3(1))) vp_3=[nan nan]; end;
           
                for sample_top_pt_id=1:size(sample_top_pts,2)
        %             yaw_esti = all_conds_scores(end,3);sample_top_pt_id = all_conds_scores(end,4);                        
                    sample_cond = [yaw_esti  sample_top_pt_id];
                    if(whether_plot_detail_img)
                        figure(figure_id_3);imshow(rgb_img);title('Config 2 Detected 2D object --> 3D');hold on;
                        rectangle('Position',[left_x_raw,top_y_raw,right_x_raw-left_x_raw,down_y_expan-top_y_raw],'EdgeColor','c','LineWidth',1); 
%                         text(obj_bbox_coors(object_id,1)+5,obj_bbox_coors(object_id,2)-10, sprintf('%s---%.2f',obj_bbox_class{1},obj_bbox_coors(object_id,5)))
                    end

                    corner_1_top = sample_top_pts(:,sample_top_pt_id)';
                    config_2_good=true;        

                    vp_1_position = 0;  % 0 initial as fail,  1  on left   2 on right      configuration 2 or 3. may be later if I change the yaw. it will be the same        
                    corner_2_top = seg_hit_boundary([vp_1 corner_1_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);        
                    if (corner_2_top(1)==-1)  % vp1-corner1 doesn't hit the right boundary. check whether hit left
                        corner_2_top = seg_hit_boundary([vp_1 corner_1_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                        if (corner_2_top(1)~=-1) % vp1-corner1 hit the left boundary   vp1 on the right
                            vp_1_position = 2;
                        end
                    else    % vp1-corner1 hit the right boundary   vp1 on the left
                        vp_1_position = 1;
                    end
                    if(whether_plot_detail_img)
                        figure(figure_id_3);plot([corner_1_top(1) corner_2_top(1)],[corner_1_top(2) corner_2_top(2)],'r','Linewidth',2.5);        
                    end
                    config_1_good = (vp_1_position>0);   % at least find one intersection for corner_2
                    if (~config_1_good)
                        if (print_details) disp('Configuration two fails at corner 2, outside segment'); end; continue;            
                    end
                    if (norm(corner_1_top-corner_2_top)<shorted_edge_thre)
                        if (print_details) disp('Configuration two fails at edge 1-2, too short');end; continue;
                    end
                    if (vp_1_position==1)   % then vp2 hit the left boundary
                        corner_3_top = seg_hit_boundary([vp_2 corner_2_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                        if(whether_plot_detail_img)
                            figure(figure_id_3);plot([corner_2_top(1) corner_3_top(1)],[corner_2_top(2) corner_3_top(2)],'g','Linewidth',2.5);
                        end
                    else   % or, then vp2 hit the right boundary
                        corner_3_top = seg_hit_boundary([vp_2 corner_2_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                        if(whether_plot_detail_img)
                            figure(figure_id_3);plot([corner_2_top(1) corner_3_top(1)],[corner_2_top(2) corner_3_top(2)],'g','Linewidth',2.5);
                        end
                    end
                    if (corner_3_top(1)==-1)
                        config_1_good = false; if (print_details) disp('Configuration two fails at corner 3, outside segment'); end;continue;
                    end
                    if (norm(corner_2_top-corner_3_top)<shorted_edge_thre)
                        if (print_details) disp('Configuration two fails at edge 2-3, too short'); end;continue;
                    end            
        %             if (whether_plot_images)  % only plot valid boxes
        %                 pause();
        %             end
                    % compute the last point in the top face
                    intersect_result=lineSegmentIntersect([vp_1 corner_3_top],[vp_2  corner_1_top],true);
                    corner_4_top=[intersect_result.intMatrixX intersect_result.intMatrixY];   % maybe outside image
                    if (whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_3);plot([corner_4_top(1) corner_1_top(1)],[corner_4_top(2) corner_1_top(2)],'g','Linewidth',2.5);
                        figure(figure_id_3);plot([corner_4_top(1) corner_3_top(1)],[corner_4_top(2) corner_3_top(2)],'r','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_4_top, [left_x_raw top_y_expan_distmap], [right_x_raw down_y_expan_distmap])) % as long as inside left-right boundary, won't change visibility
                        config_2_good=false;if (print_details)  disp('Configuration two fails at corner 4, outside box');end; continue;
                    end
                    if ( (norm(corner_3_top-corner_4_top)<shorted_edge_thre) || ((norm(corner_4_top-corner_1_top)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration two fails at edge 3-4/4-1, too short'); end; continue;
                    end
                    
                    corner_5_down = seg_hit_boundary([vp_3 corner_3_top],[left_x_raw down_y_expan right_x_raw down_y_expan]); % vp_3 is vertial in kitti
                    if(whether_plot_detail_img && plot_cube_generate_detail)            
                        figure(figure_id_3);plot([corner_3_top(1) corner_5_down(1)],[corner_3_top(2) corner_5_down(2)],'b','Linewidth',2.5);
                    end
                    if (corner_5_down(1)==-1)
                       config_2_good = false; if (print_details) disp('Configuration two fails at corner 5, outside segment'); end;  continue;                
                    end
                    if (norm(corner_3_top-corner_5_down)<shorted_edge_thre)
                        if (print_details) disp('Configuration two fails at edge 3-5, too short');  end; continue;
                    end

                    intersect_result=lineSegmentIntersect([vp_2 corner_5_down],[vp_3 corner_2_top],true);
                    corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_3);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b','Linewidth',2.5);                
                        figure(figure_id_3);plot([corner_6_down(1) corner_5_down(1)],[corner_6_down(2) corner_5_down(2)],'g','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_2_good=false; if (print_details) disp('Configuration two fails at corner 6, outside image'); end;  continue;
                    end
                    if ( (norm(corner_6_down-corner_2_top)<shorted_edge_thre) || ((norm(corner_6_down-corner_5_down)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration one fails at edge 6-5/6-2, too short'); end; continue;
                    end
                    
                    intersect_result=lineSegmentIntersect([vp_1 corner_6_down],[vp_3 corner_1_top],true);
                    corner_7_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_3);plot([corner_7_down(1) corner_1_top(1)],[corner_7_down(2) corner_1_top(2)],'b--','Linewidth',2.5);
                        figure(figure_id_3);plot([corner_7_down(1) corner_6_down(1)],[corner_7_down(2) corner_6_down(2)],'r--','Linewidth',2.5);                
                    end
                    if (~check_inside_box( corner_7_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_2_good=false; if (print_details) disp('Configuration two fails at corner 7, outside image'); end;  continue;
                    end
                    if ( (norm(corner_7_down-corner_1_top)<shorted_edge_thre) || ((norm(corner_7_down-corner_6_down)<shorted_edge_thre)) )
                        if (print_details) disp('Configuration one fails at edge 7-1/7-6, too short'); end; continue;
                    end

                    intersect_result=lineSegmentIntersect([vp_1 corner_5_down],[vp_2 corner_7_down],true);  % could also use vp3_corner4, but sometimes nearly parallel
                    corner_8_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_3);plot([corner_8_down(1) corner_4_top(1)],[corner_8_down(2) corner_4_top(2)],'b--','Linewidth',2.5);
                        figure(figure_id_3);plot([corner_8_down(1) corner_5_down(1)],[corner_8_down(2) corner_5_down(2)],'r--','Linewidth',2.5);
                        figure(figure_id_3);plot([corner_8_down(1) corner_7_down(1)],[corner_8_down(2) corner_7_down(2)],'g--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_8_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_2_good=false; if (print_details) disp('Configuration two fails at corner 8, outside image'); end;  continue;
                    end
                    if ( (norm(corner_8_down-corner_4_top)<shorted_edge_thre) || (norm(corner_8_down-corner_5_down)<shorted_edge_thre) || (norm(corner_8_down-corner_7_down)<shorted_edge_thre) )
                        if (print_details) disp('Configuration one fails at edge 8-4/8-5/8-7, too short'); end; continue;
                    end

                    box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                    box_corners_2d_int = round(box_corners_2d_float);
                    all_box_corners_2d_conf2 = [all_box_corners_2d_conf2; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]

                    if (whether_plot_detail_img && plot_cube_generate_detail)  % only plot valid boxes
                        pause(0.5);
                    end
                end
            end
            
            all_box_corners_2d_conf2_raw = [all_box_corners_2d_conf2_raw;all_box_corners_2d_conf2];
            end


            %% configuration three  can see one faces
            if (consider_config_3)
            all_box_corners_2d_conf5 =[];    
            figure_id_5 = 70;
            
            for yaw_esti = obj_yaw_samples
                [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                if (isnan(vp_3(1))) vp_3=[nan nan]; end;

                % VP1 should lie inside the cube
                if (~check_inside_box( vp_1,[left_x_raw top_y_raw], [right_x_raw down_y_expan]))
%                     [vp_1  left_x_raw top_y_raw right_x_raw down_y_expan]
                    if (print_details) disp('Configuration five, vp1 outside image'); end;  continue;
                else
%                     fprintf('vp_1 inside box %d \n',frame_index);
                end
                % find VP supported edges, boundary edges, angles and so on.  To evaluate cuboid model.
                if (whether_evaluate_cuboid)
                    all_vp_bound_edge_angles = VP_support_edge_infos([vp_1;vp_2;vp_3],edge_mid_pts,lines_inobj_angles,[vp12_edge_angle_thre vp3_edge_angle_thre]);
                end                
                vp2_position = 0;  % 0 initial as fail,  1  on left   2 on right 
                corner_1_top = [right_x_raw top_y_raw];  % first try vp2 on the left
                corner_4_top = seg_hit_boundary([corner_1_top vp_2],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                if (corner_4_top(1)~=-1)  % if hit
                    vp2_position=1;
                else
                    corner_1_top = [left_x_raw top_y_raw];
                    corner_4_top = seg_hit_boundary([corner_1_top vp_2],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                    if (corner_4_top(1)~=-1)  % if hit
                        vp2_position=2;
                    end
                end
                if (vp2_position==0)  % no any valid configurations
                    continue;
                end

                if (vp2_position==1)
                    corner_7_down = [right_x_raw down_y_expan];
                    corner_8_down = seg_hit_boundary([corner_7_down vp_2],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                    if (corner_8_down(1)==-1)    % check inside boundary. otherwise edge visibility might be wrong
                        config_5_good=false;if (print_details)  disp('Configuration five fails at corner 8, outside box');end; continue;
                    end
                else
                    corner_7_down = [left_x_raw down_y_expan];
                    corner_8_down = seg_hit_boundary([corner_7_down vp_2],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                    if (corner_8_down(1)==-1)    % check inside boundary. otherwise edge visibility might be wrong
                        config_5_good=false;if (print_details)  disp('Configuration five fails at corner 8, outside box');end; continue;
                    end
                end

                sample_top2_pts = [linspace(corner_1_top(1),vp_1(1),12);linspace(corner_1_top(2),vp_1(2),12)];
                sample_top2_pts(:,1)=[];sample_top2_pts(:,end)=[];  % or add margin                    

                for sample_top2_id = 1:size(sample_top2_pts,2)

                    if(whether_plot_detail_img)
                        figure(figure_id_5);imshow(rgb_img);title('Config 5 Detected 2D object --> 3D');hold on;
                        rectangle('Position',[left_x_raw,top_y_raw,right_x_raw-left_x_raw,down_y_expan-top_y_raw],'EdgeColor','c','LineWidth',1); 
                        text(obj_bbox_coors(object_id,1)+5,obj_bbox_coors(object_id,2)-10, sprintf('%s---%.2f',obj_bbox_class{1},obj_bbox_coors(object_id,5)))
                    end                        
                    if(whether_plot_detail_img)
                        figure(figure_id_5);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_7_down(1) corner_8_down(1)],[corner_7_down(2) corner_8_down(2)],'g','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_1_top(1) corner_7_down(1)],[corner_1_top(2) corner_7_down(2)],'b','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_4_top(1) corner_8_down(1)],[corner_4_top(2) corner_8_down(2)],'b','Linewidth',2.5);
                    end

                    corner_2_top = sample_top2_pts(:,sample_top2_id)';
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_5);plot([corner_2_top(1) corner_1_top(1)],[corner_2_top(2) corner_1_top(2)],'r--','Linewidth',2.5);
                    end

                    intersect_result = lineSegmentIntersect([vp_2 corner_2_top],[vp_1 corner_4_top],true);
                    corner_3_top=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_5);plot([corner_3_top(1) corner_2_top(1)],[corner_3_top(2) corner_2_top(2)],'g--','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_3_top(1) corner_4_top(1)],[corner_3_top(2) corner_4_top(2)],'r--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_3_top, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_5_good=false; if (print_details) disp('Configuration five fails at corner 3, outside image'); end;  continue;
                    end

                    intersect_result = lineSegmentIntersect([vp_3 corner_2_top],[vp_1 corner_7_down],true);
                    corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_5);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b--','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_6_down(1) corner_7_down(1)],[corner_6_down(2) corner_7_down(2)],'r--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_5_good=false; if (print_details) disp('Configuration five fails at corner 6, outside image'); end;  continue;
                    end

                    intersect_result = lineSegmentIntersect([vp_3 corner_3_top],[vp_1 corner_8_down],true);
                    corner_5_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                    if(whether_plot_detail_img && plot_cube_generate_detail)
                        figure(figure_id_5);plot([corner_5_down(1) corner_3_top(1)],[corner_5_down(2) corner_3_top(2)],'b--','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_5_down(1) corner_6_down(1)],[corner_5_down(2) corner_6_down(2)],'g--','Linewidth',2.5);
                        figure(figure_id_5);plot([corner_5_down(1) corner_8_down(1)],[corner_5_down(2) corner_8_down(2)],'r--','Linewidth',2.5);
                    end
                    if (~check_inside_box( corner_5_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        config_5_good=false; if (print_details) disp('Configuration five fails at corner 5, outside image'); end;  continue;
                    end

                    box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                    box_corners_2d_int = round(box_corners_2d_float);
                    all_box_corners_2d_conf5 = [all_box_corners_2d_conf5; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]

                    if (whether_plot_detail_img && plot_cube_generate_detail)  % only plot valid boxes
                        pause();
                    end
                end
            end
            
            all_box_corners_2d_conf3_raw = [all_box_corners_2d_conf3_raw;all_box_corners_2d_conf5];
            end

        end
        end
        end
        
        %% stack all cuboids of different configurations together
        if (consider_config_1)
            all_box_corners_2ds = [all_box_corners_2ds; all_box_corners_2d_conf1_raw];
        end
        if (consider_config_2)            
            all_box_corners_2ds = [all_box_corners_2ds;all_box_corners_2d_conf2_raw];
        end
        if (consider_config_3)
            all_box_corners_2ds = [all_box_corners_2ds;all_box_corners_2d_conf3_raw];
        end
        
    end

    
end
