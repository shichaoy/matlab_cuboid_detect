% main file to detect 3D cuboid from single image. Require input of 2D bouding box and camera poses.

clear;
close all;

frame_index = 0;  % choose 0 or 1

%% set some file or folder 
load('data/frame_full_infos.mat');   % load frame pose information
load('data/all_select_2d_bboxes.mat');  % load 2d bbox

rgb_folder = 'data/';
data_edge_dir = 'data/edge_detection/LSD/'; % LSD or Edline  LSD generate more edges for objects


%% set important hyper paramaters
get_relative_measures = true;         % true if set camera x=y=yaw=0, only provide height, roll, pitch,  for object SLAM cuboid coordinates should be in local frame
whether_sample_bbox_height = false;   % sample object height as raw detection might not be accurate. top fixed. sample down. affect cuboid distance
whether_sample_roll_pitch = false;    % sample camera roll pitch in case don't have/want ground truth camera pose
whether_sample_cam_height = false;    % sample camera height. top fixed, sample down.
whether_evaluate_cuboid = true;       % whether record edge distance and vanishing point edge error. used for score proposal

consider_config_1 = true;   % see three faces   false  true
consider_config_2 = true;   % see two faces      conf2/3 likely to have long thin objects
consider_config_3 = false;  % see one face   not perfect, mainly for kitti vertical obj.
canny_edge_thre = 0.10;
reweight_edge_distance = true;        % if want to compare with all configurations. we need to reweight


plot_cube_generate_detail = false; % false  true  plot each valid proposal
plot_supporting_img = false;   % plot other useful images during cuboid generation
print_message_details = false;

%% start generating cuboid proposal
disp(['Processing image ',num2str(frame_index)]);

frame_calib_mat = frame_full_infos(frame_index+1); % matlab ind start from 1

rgb_img = imread([rgb_folder sprintf('%04d_rgb_raw.jpg',frame_index)]);
if length(size(rgb_img))==2
    rgb_img = cat(3, rgb_img, rgb_img, rgb_img);
end
img_width=size(rgb_img,2);  img_height=size(rgb_img,1);

if(plot_supporting_img)
    figure(1); imshow(rgb_img);title('Raw Image');hold on;
end

% read detected long edges   I used my c++ lsd or edlines
if (whether_evaluate_cuboid)
    gray_img = rgb2gray(rgb_img);
    all_lines_raw = importdata([data_edge_dir sprintf('%04d',frame_index) '_edge.txt']);  % [x1 y1 x2 y2] raw txt is c++ index
    all_lines_raw = all_lines_raw+1; % matlab image pixel index start from 1
    all_lines_raw(all_lines_raw(:,1)>img_width,1)=img_width; all_lines_raw(all_lines_raw(:,2)>img_height,2)=img_height;
    all_lines_raw(all_lines_raw(:,3)>img_width,3)=img_width; all_lines_raw(all_lines_raw(:,4)>img_height,4)=img_height;
    all_lines_raw = align_left_right_edges(all_lines_raw);
    if(plot_supporting_img)
        plot_image_with_edges(1, rgb_img, all_lines_raw, 'Raw detected Edges', 'r', false)
    end
end

% read selected yolo 2d object bounding boxes
obj_bbox_coors = all_select_2d_bboxes{frame_index+1};
if (exist('all_select_2dboxes_classes'))
    obj_bbox_class = all_select_2dboxes_classes{frame_index+1};  % class name no use.
else
    obj_bbox_class = cell(size(obj_bbox_coors,1),1);
end

if (size(obj_bbox_coors,1)==0)    
    fprintf('Skip, Not found valid 2D object of frame %d \n',frame_index);
end


%% camera information
euler_angle = Rot_to_EulerZYX(frame_calib_mat.Rot);  % .Rot is camera rotation wrt. ground.
init_roll = euler_angle(1);
init_pitch = euler_angle(2);
if (get_relative_measures)
    frame_calib_mat.position(1:2) = 0;  % set x y=0
    euler_angle(3)=0;                   % set yaw = 0
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
Kalib = frame_calib_mat.K;  % internal calibration c++ index, might need to +1
invK = inv(Kalib);
projectionMatrix = Kalib*invT(1:3,:);  % project world coordinate to image

ground_plane_world = [0,0,1,0];  % 1*4   % in my pop-up code, I use [0 0 -1 0]. here I want the normal pointing innerwards, towards the camera to match surface normal prediction
ground_plane_sensor = ground_plane_world*transToWolrd;


%% get object cuboid samples in 3D space through VPs.
% some parameters
vp12_edge_angle_thre = 15; vp3_edge_angle_thre = 10;  % threshod of which line belong to which vp.
shorted_edge_thre = 20;  % if box edge are too short. box might be too thin. most possibly wrong.        

ObjectSets = cell(size(obj_bbox_coors,1),1);   % saved "cuboid" struct for each 2d box

for object_id = 1:size(obj_bbox_coors,1)
    left_x_raw = obj_bbox_coors(object_id,1); top_y_raw = obj_bbox_coors(object_id,2); obj_width_raw=obj_bbox_coors(object_id,3); obj_height_raw=obj_bbox_coors(object_id,4);
    right_x_raw=left_x_raw+obj_width_raw; down_y_raw = top_y_raw + obj_height_raw;    
    
    if (whether_sample_bbox_height)
        down_expand_sample_ranges = max(min(20, obj_height_raw-90),0);
        down_expand_sample_ranges = min(down_expand_sample_ranges,img_height-top_y_raw-obj_height_raw);  % should lie inside the image            
        if (down_expand_sample_ranges>10)  % should within boundaris
            down_expand_sample_all=[0 round(down_expand_sample_ranges/2) down_expand_sample_ranges];  % if expand large margin, give more samples.            
        else
            down_expand_sample_all=[0 down_expand_sample_ranges];
        end
    else
        down_expand_sample_all = [0];
    end
    
    if (whether_sample_roll_pitch)
        sample_cam_rolls_all = init_roll-deg2rad(12):deg2rad(3):init_roll+deg2rad(12);
        sample_cam_pitches_all = init_pitch-deg2rad(12):deg2rad(3):init_pitch+deg2rad(12);
    else
        sample_cam_rolls_all = [init_roll];
        sample_cam_pitches_all = [init_pitch];
    end
    
    if (whether_sample_cam_height)
        sample_cam_heights_all = frame_calib_mat.camera_height-0.4:0.2:frame_calib_mat.camera_height+0.4;
    else
        sample_cam_heights_all = [frame_calib_mat.camera_height];
    end    
    
    all_configs_errors = [];  % record proposal score/error
    all_box_corners_2ds = []; % proposal corners
    for down_expand_sample = down_expand_sample_all
        obj_height_expan = obj_height_raw + down_expand_sample;
        down_y_expan = top_y_raw + obj_height_expan;    obj_diaglength_expan = sqrt(obj_width_raw^2+obj_height_expan^2);
        
        if (plot_supporting_img)
            figure(20);imshow(rgb_img);hold on;
            hh=rectangle('Position',[left_x_raw top_y_raw,obj_width_raw,obj_height_expan],'EdgeColor',get_id_color(object_id),'LineWidth',2); hh.LineStyle='--';
            pause(0.5);
        end
        
        % sample points on the top edges, if edge is too large, give more samples. give at least 10 samples for all edges.
        top_sample_resolution=round(min(20,obj_width_raw/10 )); %  at most 20 pixels per sample
        sample_top_pts = left_x_raw+5:top_sample_resolution:right_x_raw-5;sample_top_pts=[sample_top_pts;top_y_raw*ones(1,size(sample_top_pts,2))];
        
        % expand some small margin for distance map
        distmap_expand_wid = min(max(min(20, obj_width_raw-100),10),max(min(20, obj_height_expan-100),10)); 
        left_x_expan_distmap = max(1,left_x_raw-distmap_expand_wid);right_x_expan_distmap = min(img_width,right_x_raw+distmap_expand_wid);
        top_y_expan_distmap = max(1,top_y_raw-distmap_expand_wid);down_y_expan_distmap = min(img_height,down_y_expan+distmap_expand_wid);

        
        if (whether_evaluate_cuboid)  % get canny edge, build distance transform. find long edges inside 2d object box.
            all_lines_inside_object=[];
            for edge_id=1:size(all_lines_raw,1)
                if (check_inside_box( all_lines_raw(edge_id,1:2), [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                    if (check_inside_box( all_lines_raw(edge_id,3:4), [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        all_lines_inside_object=[all_lines_inside_object; all_lines_raw(edge_id,:)];
                    end
                end
            end
            if(plot_supporting_img)
                plot_image_with_edges(30, rgb_img, all_lines_inside_object, 'Raw Inside Object Edges', 'r', true);pause(0.5);
            end
            % merge short edges
            pre_merge_dist_thre = 20; pre_merge_angle_thre = 5;
            all_lines_merge_inobj = merge_break_lines_v2(all_lines_inside_object,pre_merge_dist_thre,pre_merge_angle_thre);
            edge_length_threshold = 30;  % raw edge is thresholded by 20
            all_lines_merge_inobj = remove_short_lines(all_lines_merge_inobj,edge_length_threshold);
            if(plot_supporting_img)
                plot_image_with_edges(35, rgb_img, all_lines_merge_inobj, 'Merged Inside Object Edges', 'r', true);pause(0.5);
            end

            % compute edge angels and middle points
            if size(all_lines_merge_inobj,1)>0
                lines_inobj_angles = atan2(all_lines_merge_inobj(:,4)-all_lines_merge_inobj(:,2), abs(all_lines_merge_inobj(:,3)-all_lines_merge_inobj(:,1)));   % [-pi/2 +pi/2]
                edge_mid_pts = [(all_lines_merge_inobj(:,1)+all_lines_merge_inobj(:,3))/2  (all_lines_merge_inobj(:,2)+all_lines_merge_inobj(:,4))/2];
            else
                lines_inobj_angles=[];edge_mid_pts=[];
            end
            
            % detect cannny edge, and compute distance transform % TODO whether smooth before canny edge.
            im_canny = edge(gray_img, 'canny',canny_edge_thre); % 0.10  or let it choose automatically
            if (plot_supporting_img)
                figure(59);imshow(im_canny);title('Whole Image canny');pause(0.5);
            end        
            im_canny(1:top_y_expan_distmap-1,:)=0;im_canny(down_y_expan_distmap+1:end,:)=0;  % only consider edges inside object bounding box. but outside edges could also be bad.!!!
            im_canny(:,1:left_x_expan_distmap-1)=0;im_canny(:,right_x_expan_distmap+1:end)=0;
            if (plot_supporting_img)
                figure(52);imshow(im_canny);title('Object area canny');pause(0.5);
            end
            dist_map = bwdist(im_canny);   % could be some speed up. for example only consider the object area
            dist_map(1:top_y_expan_distmap-1,:)=0;dist_map(down_y_expan_distmap+1:end,:)=0;
            dist_map(:,1:left_x_expan_distmap-1)=0;dist_map(:,right_x_expan_distmap+1:end)=0;
            dist_map_img = repmat(mat2gray(dist_map), [1 1 3]);
            if (plot_supporting_img)        
                figure(54);imshow(dist_map_img), title('Euclidean distance map');pause(0.5);
            end
        end
        
        
        yaw_init = camera_yaw-90/180*pi;  % object yaw initialized to face camera!!!

        all_configs_error_conf1_raw = [];    all_box_corners_2d_conf1_raw =[];  % record corners and error for each configuration
        all_configs_error_conf2_raw = [];    all_box_corners_2d_conf2_raw =[];
        all_configs_error_conf3_raw = [];    all_box_corners_2d_conf3_raw =[];
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

            if(plot_supporting_img)
                % compute initial vanishing points
                [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_init); % for object x y z  axis                    
                figure(25);clf;title('detected Vps');hold on;
                imagesc(rgb_img);hold on;
                plot(vp_1(1),vp_1(2),'r*','MarkerSize',20);
                plot(vp_2(1),vp_2(2),'g*','MarkerSize',20);
                plot(vp_3(1),vp_3(2),'b*','MarkerSize',20);hold off;
                axis ij;axis equal;pause(0.5);
            end                        
            
            obj_yaw_samples = yaw_init-45/180*pi:6/180*pi:yaw_init+45/180*pi;  % search an object raw range of 90

            %% configuration one  can see three faces
            if (consider_config_1)
                all_configs_error_conf1 = [];    all_box_corners_2d_conf1 =[];
                evaluation_method = 2;   % 1 for surface normal (needs CNN prediction)  2 for  distance transform of canny pixels   3 for VP supported edges.            

                figure_id_1 = 60;            
                for yaw_sample_id = 1:size(obj_yaw_samples,2)
                    yaw_esti = obj_yaw_samples(yaw_sample_id);
                    [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                    if (isnan(vp_3(1))) vp_3=[nan nan]; end;

                    % find VP supported edges, boundary edges, angles and so on.  To evaluate cuboid model.
                    if (whether_evaluate_cuboid)
                        all_vp_bound_edge_angles = VP_support_edge_infos([vp_1;vp_2;vp_3],edge_mid_pts,lines_inobj_angles,[vp12_edge_angle_thre vp3_edge_angle_thre]);
                    end
                    
                    for sample_top_pt_id=1:size(sample_top_pts,2)
                        sample_cond = [yaw_sample_id  sample_top_pt_id];
                        if(plot_cube_generate_detail)
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
                        if(plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_1_top(1) corner_2_top(1)],[corner_1_top(2) corner_2_top(2)],'r','Linewidth',2.5);        
                        end        
                        config_1_good = (vp_1_position>0);   % at least find one intersection for corner_2
                        if (~config_1_good)
                            if (print_message_details) disp('Configuration one fails at corner 2, outside segment'); end;  continue;
                        end
                        if (norm(corner_1_top-corner_2_top)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration one fails at edge 1-2, too short');  end;  continue;
                        end

                        if (vp_1_position==1)   % then [vp2 pt1] hit the left boundary
                            corner_4_top = seg_hit_boundary([vp_2 corner_1_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                            if(plot_cube_generate_detail)
                                figure(figure_id_1);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                            end
                        else   % then [vp2 pt1] hit the right boundary
                            corner_4_top = seg_hit_boundary([vp_2 corner_1_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                            if(plot_cube_generate_detail)
                                figure(figure_id_1);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                            end
                        end

                        if (corner_4_top(1)==-1)
                            config_1_good = false; if (print_message_details)  disp('Configuration one fails at corner 4, outside segment'); end;  continue;
                        end
                        if (norm(corner_1_top-corner_4_top)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration one fails at edge 1-4, too short'); end;  continue;
                        end

                        % compute the last point in the top face
                        intersect_result=lineSegmentIntersect([vp_2  corner_2_top],[vp_1 corner_4_top],true);
                        corner_3_top=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_3_top(1) corner_2_top(1)],[corner_3_top(2) corner_2_top(2)],'g','Linewidth',2.5);
                            figure(figure_id_1);plot([corner_3_top(1) corner_4_top(1)],[corner_3_top(2) corner_4_top(2)],'r','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_3_top, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))    % check inside boundary. otherwise edge visibility might be wrong           
                            config_1_good=false;  if (print_message_details) disp('Configuration one fails at corner 3, outside box'); end;  continue;
                        end
                        if ( (norm(corner_3_top-corner_4_top)<shorted_edge_thre) || ((norm(corner_3_top-corner_2_top)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration one fails at edge 3-4/3-2, too short'); end;  continue;
                        end
                        % compute first bottom points
                        corner_5_down = seg_hit_boundary([vp_3 corner_3_top],[left_x_raw down_y_expan right_x_raw down_y_expan]); % vp_3 is vertial in kitti
                        if(plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_3_top(1) corner_5_down(1)],[corner_3_top(2) corner_5_down(2)],'b','Linewidth',2.5);
                        end
                        if (corner_5_down(1)==-1)
                           config_1_good = false; if (print_message_details)  disp('Configuration one fails at corner 5, outside segment'); end; continue;
                        end
                        if (norm(corner_3_top-corner_5_down)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration one fails at edge 3-5, too short'); end;  continue;
                        end
                        % don't need to check bottom points, it actually includes some other configurations...
                        intersect_result=lineSegmentIntersect([vp_2 corner_5_down],[vp_3 corner_2_top],true);
                        corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b','Linewidth',2.5);
                            figure(figure_id_1);plot([corner_6_down(1) corner_5_down(1)],[corner_6_down(2) corner_5_down(2)],'g','Linewidth',2.5);                
                        end
                        if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
    %                     if (~check_inside_box( corner_6_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))
                            config_1_good=false; if (print_message_details) disp('Configuration one fails at corner 6, outside image'); end; continue;
                        end
                        if ( (norm(corner_6_down-corner_2_top)<shorted_edge_thre) || ((norm(corner_6_down-corner_5_down)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration one fails at edge 6-5/6-2, too short'); end; continue;
                        end
                        intersect_result=lineSegmentIntersect([vp_1 corner_6_down],[vp_3 corner_1_top],true);
                        corner_7_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_1);plot([corner_7_down(1) corner_1_top(1)],[corner_7_down(2) corner_1_top(2)],'b--','Linewidth',2.5);                
                            figure(figure_id_1);plot([corner_7_down(1) corner_6_down(1)],[corner_7_down(2) corner_6_down(2)],'r--','Linewidth',2.5);
                        end
%                         if (~check_inside_box( corner_7_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                        if (~check_inside_box( corner_7_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))  % 7 must line inside object. hidden point                              
                            config_1_good=false; if (print_message_details) disp('Configuration one fails at corner 7, outside image'); end; continue;
                        end
                        if ( (norm(corner_7_down-corner_1_top)<shorted_edge_thre) || ((norm(corner_7_down-corner_6_down)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration one fails at edge 7-1/7-6, too short'); end; continue;
                        end            
                        intersect_result=lineSegmentIntersect([vp_1 corner_5_down],[vp_2 corner_7_down],true);  % could also use vp3_corner4, but sometimes nearly parallel
                        corner_8_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)                
                            figure(figure_id_1);plot([corner_8_down(1) corner_4_top(1)],[corner_8_down(2) corner_4_top(2)],'b','Linewidth',2.5);
                            figure(figure_id_1);plot([corner_8_down(1) corner_5_down(1)],[corner_8_down(2) corner_5_down(2)],'r','Linewidth',2.5);
                            figure(figure_id_1);plot([corner_8_down(1) corner_7_down(1)],[corner_8_down(2) corner_7_down(2)],'g--','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_8_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
%                         if (~check_inside_box( corner_8_down, [left_x_raw top_y_raw], [right_x_raw down_y_expan]))                                
                            config_1_good=false; if (print_message_details) disp('Configuration one fails at corner 8, outside image');end;  continue;
                        end
                        if ( (norm(corner_8_down-corner_4_top)<shorted_edge_thre) || (norm(corner_8_down-corner_5_down)<shorted_edge_thre) || (norm(corner_8_down-corner_7_down)<shorted_edge_thre) )
                            if (print_message_details) disp('Configuration one fails at edge 8-4/8-5/8-7, too short'); end; continue;
                        end

                        box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                        all_box_corners_2d_conf1 = [all_box_corners_2d_conf1; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]

                %         Evaluation method II      use distance transform from canny detected pixels. also called Chamfer distance.
                        if (evaluation_method==2)
                            sum_dist = 0; total_angle_diff =0;
                            if (whether_evaluate_cuboid)
                                visible_edge_pt_ids = [1 2;2 3;3 4;4 1;2 6;3 5;4 8;5 8;5 6];
                                sum_dist = box_edge_sum_dists(dist_map,box_corners_2d_float,visible_edge_pt_ids);  % sample points, no other parameters   detect ourlier using max_distance?

                                vps_box_edge_pt_ids = [1 2 8 5;4 1 5 6;4 8 2 6]; % six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
                                total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles,vps_box_edge_pt_ids,box_corners_2d_float);
                            end
                            all_configs_error_conf1 = [all_configs_error_conf1;1 vp_1_position yaw_esti sample_top_pt_id sum_dist/obj_diaglength_expan total_angle_diff ...
                                        down_expand_sample rad2deg(sample_cam_roll-init_roll) rad2deg(sample_cam_pitch-init_pitch) sample_cam_height]; % 1 means configuration 1
                        end

                        if (plot_cube_generate_detail)  % only plot valid boxes
                            pause(0.5);
                        end
                    end
                end
                all_configs_error_conf1_raw = [all_configs_error_conf1_raw; all_configs_error_conf1];            
                all_box_corners_2d_conf1_raw = [all_box_corners_2d_conf1_raw; all_box_corners_2d_conf1];            
            end

            
            %% configuration two  can see two faces
            if (consider_config_2)
                all_configs_error_conf2 = [];    all_box_corners_2d_conf2 =[];
                figure_id_3 = 70;
                % the top four points are generatedly differently, the bottom four are same, except with different visibility
                for yaw_sample_id = 1:size(obj_yaw_samples,2)
                    yaw_esti = obj_yaw_samples(yaw_sample_id);
                    [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                    if (isnan(vp_3(1))) vp_3=[nan nan]; end;
                    
                    % find VP supported edges, boundary edges, angles and so on.  To evaluate cuboid model.
                    if (whether_evaluate_cuboid)
                        all_vp_bound_edge_angles = VP_support_edge_infos([vp_1;vp_2;vp_3],edge_mid_pts,lines_inobj_angles,[vp12_edge_angle_thre vp3_edge_angle_thre]);
                    end
                    for sample_top_pt_id=1:size(sample_top_pts,2)
                        sample_cond = [yaw_esti  sample_top_pt_id];
                        if(plot_cube_generate_detail)
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
                        if(plot_cube_generate_detail)
                            figure(figure_id_3);plot([corner_1_top(1) corner_2_top(1)],[corner_1_top(2) corner_2_top(2)],'r','Linewidth',2.5);        
                        end
                        config_2_good = (vp_1_position>0);   % at least find one intersection for corner_2
                        if (~config_2_good)
                            if (print_message_details) disp('Configuration two fails at corner 2, outside segment'); end; continue;            
                        end
                        if (norm(corner_1_top-corner_2_top)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration two fails at edge 1-2, too short');end; continue;
                        end
                        if (vp_1_position==1)   % then [vp2 pt2] hit the left boundary
                            corner_3_top = seg_hit_boundary([vp_2 corner_2_top],[left_x_raw top_y_raw left_x_raw down_y_expan]);
                            if(plot_cube_generate_detail)
                                figure(figure_id_3);plot([corner_2_top(1) corner_3_top(1)],[corner_2_top(2) corner_3_top(2)],'g','Linewidth',2.5);
                            end
                        else   % then [vp2 pt2] hit the right boundary
                            corner_3_top = seg_hit_boundary([vp_2 corner_2_top],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                            if(plot_cube_generate_detail)
                                figure(figure_id_3);plot([corner_2_top(1) corner_3_top(1)],[corner_2_top(2) corner_3_top(2)],'g','Linewidth',2.5);
                            end
                        end
                        if (corner_3_top(1)==-1)
                            config_2_good = false; if (print_message_details) disp('Configuration two fails at corner 3, outside segment'); end;continue;
                        end
                        if (norm(corner_2_top-corner_3_top)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration two fails at edge 2-3, too short'); end;continue;
                        end            
            %             if (whether_plot_images)  % only plot valid boxes
            %                 pause();
            %             end
                        % compute the last point in the top face
                        intersect_result=lineSegmentIntersect([vp_1 corner_3_top],[vp_2  corner_1_top],true);
                        corner_4_top=[intersect_result.intMatrixX intersect_result.intMatrixY];   % maybe outside image
                        if (plot_cube_generate_detail)
                            figure(figure_id_3);plot([corner_4_top(1) corner_1_top(1)],[corner_4_top(2) corner_1_top(2)],'g','Linewidth',2.5);
                            figure(figure_id_3);plot([corner_4_top(1) corner_3_top(1)],[corner_4_top(2) corner_3_top(2)],'r','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_4_top, [left_x_raw top_y_expan_distmap], [right_x_raw down_y_expan_distmap])) % as long as inside left-right boundary, won't change visibility
                            config_2_good=false;if (print_message_details)  disp('Configuration two fails at corner 4, outside box');end; continue;
                        end
                        if ( (norm(corner_3_top-corner_4_top)<shorted_edge_thre) || ((norm(corner_4_top-corner_1_top)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration two fails at edge 3-4/4-1, too short'); end; continue;
                        end
                        % NOTE! the following code is same as config 1
                        corner_5_down = seg_hit_boundary([vp_3 corner_3_top],[left_x_raw down_y_expan right_x_raw down_y_expan]); % vp_3 is vertial in kitti
                        if(plot_cube_generate_detail)            
                            figure(figure_id_3);plot([corner_3_top(1) corner_5_down(1)],[corner_3_top(2) corner_5_down(2)],'b','Linewidth',2.5);
                        end
                        if (corner_5_down(1)==-1)
                           config_2_good = false; if (print_message_details) disp('Configuration two fails at corner 5, outside segment'); end;  continue;                
                        end
                        if (norm(corner_3_top-corner_5_down)<shorted_edge_thre)
                            if (print_message_details) disp('Configuration two fails at edge 3-5, too short');  end; continue;
                        end

                        intersect_result=lineSegmentIntersect([vp_2 corner_5_down],[vp_3 corner_2_top],true);
                        corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_3);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b','Linewidth',2.5);                
                            figure(figure_id_3);plot([corner_6_down(1) corner_5_down(1)],[corner_6_down(2) corner_5_down(2)],'g','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_2_good=false; if (print_message_details) disp('Configuration two fails at corner 6, outside image'); end;  continue;
                        end
                        if ( (norm(corner_6_down-corner_2_top)<shorted_edge_thre) || ((norm(corner_6_down-corner_5_down)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration one fails at edge 6-5/6-2, too short'); end; continue;
                        end

                        intersect_result=lineSegmentIntersect([vp_1 corner_6_down],[vp_3 corner_1_top],true);
                        corner_7_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_3);plot([corner_7_down(1) corner_1_top(1)],[corner_7_down(2) corner_1_top(2)],'b--','Linewidth',2.5);
                            figure(figure_id_3);plot([corner_7_down(1) corner_6_down(1)],[corner_7_down(2) corner_6_down(2)],'r--','Linewidth',2.5);                
                        end
                        if (~check_inside_box( corner_7_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_2_good=false; if (print_message_details) disp('Configuration two fails at corner 7, outside image'); end;  continue;
                        end
                        if ( (norm(corner_7_down-corner_1_top)<shorted_edge_thre) || ((norm(corner_7_down-corner_6_down)<shorted_edge_thre)) )
                            if (print_message_details) disp('Configuration one fails at edge 7-1/7-6, too short'); end; continue;
                        end

                        intersect_result=lineSegmentIntersect([vp_1 corner_5_down],[vp_2 corner_7_down],true);  % could also use vp3_corner4, but sometimes nearly parallel
                        corner_8_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_3);plot([corner_8_down(1) corner_4_top(1)],[corner_8_down(2) corner_4_top(2)],'b--','Linewidth',2.5);
                            figure(figure_id_3);plot([corner_8_down(1) corner_5_down(1)],[corner_8_down(2) corner_5_down(2)],'r--','Linewidth',2.5);
                            figure(figure_id_3);plot([corner_8_down(1) corner_7_down(1)],[corner_8_down(2) corner_7_down(2)],'g--','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_8_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_2_good=false; if (print_message_details) disp('Configuration two fails at corner 8, outside image'); end;  continue;
                        end
                        if ( (norm(corner_8_down-corner_4_top)<shorted_edge_thre) || (norm(corner_8_down-corner_5_down)<shorted_edge_thre) || (norm(corner_8_down-corner_7_down)<shorted_edge_thre) )
                            if (print_message_details) disp('Configuration one fails at edge 8-4/8-5/8-7, too short'); end; continue;
                        end

                        box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                        all_box_corners_2d_conf2 = [all_box_corners_2d_conf2; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]
                        
                        if (evaluation_method==2)
                            sum_dist = 0; total_angle_diff =0;
                            if (whether_evaluate_cuboid)
                                visible_edge_pt_ids = [1 2;2 3;3 4;4 1;2 6;3 5;5 6];
                                sum_dist = box_edge_sum_dists(dist_map,box_corners_2d_float,visible_edge_pt_ids,reweight_edge_distance);

                                vps_box_edge_pt_ids = [1 2 3 4;4 1 5 6;3 5 2 6]; % six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
                                total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles,vps_box_edge_pt_ids,box_corners_2d_float);
                            end                        
                            all_configs_error_conf2 = [all_configs_error_conf2;2 vp_1_position yaw_esti sample_top_pt_id sum_dist/obj_diaglength_expan total_angle_diff ...
                                    down_expand_sample rad2deg(sample_cam_roll-init_roll) rad2deg(sample_cam_pitch-init_pitch) sample_cam_height]; % 2 means configuration 2
                        end
                        if (plot_cube_generate_detail)  % only plot valid boxes
                            pause(0.5);
                        end
                    end
                end
                all_configs_error_conf2_raw = [all_configs_error_conf2_raw;all_configs_error_conf2];
                all_box_corners_2d_conf2_raw = [all_box_corners_2d_conf2_raw;all_box_corners_2d_conf2];
            end


            %% configuration three  can see one faces
            if (consider_config_3)
                all_configs_error_conf3 = [];    all_box_corners_2d_conf5 =[];
                figure_id_5 = 70;

                for yaw_esti = obj_yaw_samples
                    [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw_esti); % for object x y z  axis
                    if (isnan(vp_3(1))) vp_3=[nan nan]; end;

                    % VP1 should lie inside the cube
                    if (~check_inside_box( vp_1,[left_x_raw top_y_raw], [right_x_raw down_y_expan]))
                        if (print_message_details) disp('Configuration five, vp1 outside image'); end;  continue;
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
                            config_3_good=false;if (print_message_details)  disp('Configuration five fails at corner 8, outside box');end; continue;
                        end
                    else
                        corner_7_down = [left_x_raw down_y_expan];
                        corner_8_down = seg_hit_boundary([corner_7_down vp_2],[right_x_raw top_y_raw right_x_raw down_y_expan]);
                        if (corner_8_down(1)==-1)    % check inside boundary. otherwise edge visibility might be wrong
                            config_3_good=false;if (print_message_details)  disp('Configuration five fails at corner 8, outside box');end; continue;
                        end
                    end

                    sample_top2_pts = [linspace(corner_1_top(1),vp_1(1),12);linspace(corner_1_top(2),vp_1(2),12)];
                    sample_top2_pts(:,1)=[];sample_top2_pts(:,end)=[];  % or add margin                    

                    for sample_top2_id = 1:size(sample_top2_pts,2)

                        if(plot_cube_generate_detail)
                            figure(figure_id_5);imshow(rgb_img);title('Config 5 Detected 2D object --> 3D');hold on;
                            rectangle('Position',[left_x_raw,top_y_raw,right_x_raw-left_x_raw,down_y_expan-top_y_raw],'EdgeColor','c','LineWidth',1); 
                            text(obj_bbox_coors(object_id,1)+5,obj_bbox_coors(object_id,2)-10, sprintf('%s---%.2f',obj_bbox_class{1},obj_bbox_coors(object_id,5)))
                        end                        
                        if(plot_cube_generate_detail)
                            figure(figure_id_5);plot([corner_1_top(1) corner_4_top(1)],[corner_1_top(2) corner_4_top(2)],'g','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_7_down(1) corner_8_down(1)],[corner_7_down(2) corner_8_down(2)],'g','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_1_top(1) corner_7_down(1)],[corner_1_top(2) corner_7_down(2)],'b','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_4_top(1) corner_8_down(1)],[corner_4_top(2) corner_8_down(2)],'b','Linewidth',2.5);
                        end

                        corner_2_top = sample_top2_pts(:,sample_top2_id)';
                        if(plot_cube_generate_detail)
                            figure(figure_id_5);plot([corner_2_top(1) corner_1_top(1)],[corner_2_top(2) corner_1_top(2)],'r--','Linewidth',2.5);
                        end

                        intersect_result = lineSegmentIntersect([vp_2 corner_2_top],[vp_1 corner_4_top],true);
                        corner_3_top=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_5);plot([corner_3_top(1) corner_2_top(1)],[corner_3_top(2) corner_2_top(2)],'g--','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_3_top(1) corner_4_top(1)],[corner_3_top(2) corner_4_top(2)],'r--','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_3_top, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_3_good=false; if (print_message_details) disp('Configuration five fails at corner 3, outside image'); end;  continue;
                        end

                        intersect_result = lineSegmentIntersect([vp_3 corner_2_top],[vp_1 corner_7_down],true);
                        corner_6_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_5);plot([corner_6_down(1) corner_2_top(1)],[corner_6_down(2) corner_2_top(2)],'b--','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_6_down(1) corner_7_down(1)],[corner_6_down(2) corner_7_down(2)],'r--','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_6_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_3_good=false; if (print_message_details) disp('Configuration five fails at corner 6, outside image'); end;  continue;
                        end

                        intersect_result = lineSegmentIntersect([vp_3 corner_3_top],[vp_1 corner_8_down],true);
                        corner_5_down=[intersect_result.intMatrixX intersect_result.intMatrixY];
                        if(plot_cube_generate_detail)
                            figure(figure_id_5);plot([corner_5_down(1) corner_3_top(1)],[corner_5_down(2) corner_3_top(2)],'b--','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_5_down(1) corner_6_down(1)],[corner_5_down(2) corner_6_down(2)],'g--','Linewidth',2.5);
                            figure(figure_id_5);plot([corner_5_down(1) corner_8_down(1)],[corner_5_down(2) corner_8_down(2)],'r--','Linewidth',2.5);
                        end
                        if (~check_inside_box( corner_5_down, [left_x_expan_distmap top_y_expan_distmap], [right_x_expan_distmap down_y_expan_distmap]))
                            config_3_good=false; if (print_message_details) disp('Configuration five fails at corner 5, outside image'); end;  continue;
                        end

                        box_corners_2d_float = [corner_1_top' corner_2_top' corner_3_top' corner_4_top' corner_5_down' corner_6_down' corner_7_down' corner_8_down']; % stack all corners for later use  2*8
                        box_corners_2d_int = round(box_corners_2d_float);
                        all_box_corners_2d_conf5 = [all_box_corners_2d_conf5; reshape(box_corners_2d_float,1,16)];  % according to matlab, [x1 y1 x2 y2...]
                        
                %         Evaluation method II      use distance transform from canny detected pixels. also called Chamfer distance.
                        if (evaluation_method==2)
                            sum_dist = 0; total_angle_diff =0;
                            if (whether_evaluate_cuboid)                    
                                visible_edge_pt_ids = [1 2;2 3;3 4;4 1;2 6;3 5;5 6];
                                sum_dist = box_edge_sum_dists(dist_map,box_corners_2d_float,visible_edge_pt_ids,reweight_edge_distance);

                                vps_box_edge_pt_ids = [1 2 3 4;4 1 5 6;3 5 2 6]; % six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
                                total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles,vps_box_edge_pt_ids,box_corners_2d_float);            
                            end
                            all_configs_error_conf3 = [all_configs_error_conf3;5 vp2_position yaw_esti sample_top_pt_id sum_dist/obj_diaglength_expan total_angle_diff ...
                                        down_expand_sample rad2deg(sample_cam_roll-init_roll) rad2deg(sample_cam_pitch-init_pitch) sample_cam_height];
                        end

                        if (plot_cube_generate_detail)  % only plot valid boxes
                            pause();
                        end
                    end
                end

                all_configs_error_conf3_raw = [all_configs_error_conf3_raw;all_configs_error_conf3];
                all_box_corners_2d_conf3_raw = [all_box_corners_2d_conf3_raw;all_box_corners_2d_conf5];
            end

        end
        end
        end
        
        %% stack different configurations together. sort.   config1 edge distance error is already re-weighted
        if (consider_config_1)
            all_configs_errors = [all_configs_errors; all_configs_error_conf1_raw];
            all_box_corners_2ds = [all_box_corners_2ds; all_box_corners_2d_conf1_raw];
        end
        if (consider_config_2)
            all_configs_errors = [all_configs_errors;all_configs_error_conf2_raw];
            all_box_corners_2ds = [all_box_corners_2ds;all_box_corners_2d_conf2_raw];
        end
        if (consider_config_3)
            all_configs_errors = [all_configs_errors;all_configs_error_conf3_raw];
            all_box_corners_2ds = [all_box_corners_2ds;all_box_corners_2d_conf3_raw];
        end
        
    end

    % change 2d corners into 3D cuboid struct
    raw_object_errors = [];
    for box_id=1:size(all_box_corners_2ds,1)
        sample_cam_roll = deg2rad(all_configs_errors(box_id,8)) + init_roll;
        sample_cam_pitch = deg2rad(all_configs_errors(box_id,9)) + init_pitch; 
        transToWolrd(1:3,1:3) = EulerZYX_to_Rot([sample_cam_roll sample_cam_pitch camera_yaw]);
        sample_cam_height = all_configs_errors(box_id,10);
        transToWolrd(3,4) = sample_cam_height; 
        rotationToWorld = transToWolrd(1:3,1:3);
        invT = inv(transToWolrd);
        invR = inv(rotationToWorld);
        projectionMatrix = Kalib*invT(1:3,:);  % project world coordinate to camera
        ground_plane_sensor = ground_plane_world*transToWolrd;

        new_cuboid = change_2d_corner_to_3d_object(reshape(all_box_corners_2ds(box_id,:),2,8), all_configs_errors(box_id,1:3), ground_plane_sensor, transToWolrd, invK, ...
                                                    projectionMatrix,false);
        if (size(new_cuboid,1)==0)  % if valid object. no assert error 
            continue;
        end
        skew_ratio = max(new_cuboid.scale(1:2))/min(new_cuboid.scale(1:2));  % if higher than 3, give some penalty
        
        % remove some invalid cuboid
        if any(new_cuboid.scale<0)     continue;        end                  % scale should be positive
        if any(new_cuboid.box_corners_3d_world(2,:))<0   continue;    end    % in front of camera
        if any(new_cuboid.box_corners_3d_world(3,:))<0   continue;     end;  % above ground

        new_cuboid.rect_detect_2d = [left_x_raw top_y_raw obj_width_raw obj_height_expan]; % 2D bounding box (might be expanded by me)
        new_cuboid.confidence_2d_prob = obj_bbox_coors(object_id,5);   % probability
        new_cuboid.edge_distance_error = all_configs_errors(box_id,5); % record the original error, lower is better     not score!!!
        new_cuboid.edge_angle_error = all_configs_errors(box_id,6);
        new_cuboid.skew_ratio = skew_ratio;
        new_cuboid.normalized_error = 0;
        new_cuboid.clique_id = object_id;  % object instance ID
        new_cuboid.id_in_clique = size(ObjectSets{object_id},1)+1;   % box id only belonging to this object
        new_cuboid.down_expand_height = all_configs_errors(box_id,7);
        new_cuboid.camera_roll_delta = all_configs_errors(box_id,8);
        new_cuboid.camera_pitch_delta = all_configs_errors(box_id,9);
        new_cuboid.camera_height_delta = all_configs_errors(box_id,10)-frame_calib_mat.camera_height;
        new_cuboid.mymodel = 1;
        new_cuboid.class = obj_bbox_class{object_id};
        ObjectSets{object_id} = [ObjectSets{object_id};new_cuboid];
    end

end
ObjectSets_rawproposals = ObjectSets;








%% score proposals. could also put in different file.
whether_plot_topN_img = true;
whether_plot_best_img = true;
save_plot_to_img = false;
saved_img_result_overview_dir = 'data/saved_img/'; saved_img_result_topN_dir = 'data/saved_img/'; % if want to save image.
max_cuboid_num = 15;   % get top N proposals for each object

whether_normalize_two_errors = true; weight_vp_angle = 0.8; weight_skew_error = 1.5;
% collect all errors, tune weights, normalize error and so on, finally rank.
for object_id = 1:size(ObjectSets,1)
    if (size(ObjectSets{object_id},1)>0)

        raw_object_errors = [extractfield(ObjectSets{object_id},'edge_distance_error')' extractfield(ObjectSets{object_id},'edge_angle_error')' extractfield(ObjectSets{object_id},'skew_ratio')'];

        nominal_skew_ratio = 1; maximum_skew_ratio = 2;  % need to adapt to different objects, such as bicycles.

        raw_object_errors(:,3) = max(raw_object_errors(:,3)-nominal_skew_ratio,0);  % weight 1.5   s-1  is the best found for sun rgbd object
        raw_object_errors(raw_object_errors(:,3)>maximum_skew_ratio,3) = 100; % manually add large penalty for large skew objs

        all_expan_heights = extractfield(ObjectSets{object_id},'down_expand_height')';  % object 2d box height might be sampled.
        unique_height = unique(all_expan_heights);

        % step one. fuse edge angle and distance error. remove some bad
        new_ObjectSets = [];
        new_obj_normalized_errors = [];
        for height_cond = 1:length(unique_height)
            hei_ObjectSets = ObjectSets{object_id}(all_expan_heights==unique_height(height_cond));
            hei_raw_object_errors = raw_object_errors(all_expan_heights==unique_height(height_cond),:);
            [combined_edge_angle_error,all_delete_inds] = fuse_normalize_scores(hei_raw_object_errors(:,1),hei_raw_object_errors(:,2),weight_vp_angle,whether_normalize_two_errors);
            hei_ObjectSets(all_delete_inds) = [];
            hei_raw_object_errors(all_delete_inds,:) = [];
            new_ObjectSets=[new_ObjectSets;hei_ObjectSets];
            new_obj_normalized_errors=[new_obj_normalized_errors;[combined_edge_angle_error hei_raw_object_errors(:,3)]];
        end

        %finally rank [normalized_error   skew_error]
        if (size(new_obj_normalized_errors,1)>1)
            combined_error = new_obj_normalized_errors(:,1)+weight_skew_error*new_obj_normalized_errors(:,2);
            [sorted_error, sorted_inds] = sort(combined_error,'ascend');  % rank all proposals by ascending error
            actual_cuboid_num = min(max_cuboid_num, size(sorted_error,1) );
            ObjectSets{object_id} = new_ObjectSets(sorted_inds(1:actual_cuboid_num));
            for cube_id = 1:length(ObjectSets{object_id})
                ObjectSets{object_id}(cube_id).normalized_error = sorted_error(cube_id);
                if (isnan(ObjectSets{object_id}(cube_id).normalized_error))
                    ObjectSets{object_id}(cube_id).normalized_error = 0.5; % nan is due to the same object error. cannot normalize
                end
            end
        end
    end
end

% plot final selected 9 cuboids for this object
if (whether_plot_topN_img)
    for object_id = 1:size(ObjectSets,1)
        if (size(ObjectSets{object_id},1)>0)
            figure_2d_id = 80;
            figure(figure_2d_id);clf;title('first frame');
            subplot = @(m,n,p) subtightplot (m, n, p, [0.03 0.03], [0.05 0.05], [0.05 0.05]);
            for model_id = 1:min(9,size(ObjectSets{object_id},1))
                cuboid = ObjectSets{object_id}(model_id);
                figure(figure_2d_id);subplot(3,3,model_id);imshow(rgb_img); title(sprintf('%.2f - %.2f',cuboid.edge_distance_error,cuboid.edge_angle_error));hold on;
                plot_image_with_cuboids(cuboid);   % can recover anything from "cuboid" struct
            end
            pause(0.5);
            if (save_plot_to_img)
                fig = figure(figure_2d_id);fig.Position=[100 220 1127 840];
                edge2d_img_title = sprintf('%d, Obj-%d, Type-1',frame_index, object_id);
                saved_img_name=[saved_img_result_topN_dir sprintf('%04d_obj_%d_type_1',frame_index,object_id)];
                save_figure_to_img(figure_2d_id, edge2d_img_title, saved_img_name,true);                        
            end
        end
    end
end

% plot the best cuboid for this object
if (whether_plot_best_img)
    figure_2d_id = 30;            
    title_string = sprintf('Frame %d,  best objects',frame_index);
    figure(figure_2d_id);clf;imshow(rgb_img);title(title_string);hold on;
    for object_id=1:size(ObjectSets,1)
        if ( size(ObjectSets{object_id},1)>0 )
            cuboid = ObjectSets{object_id}(1); % first one is the best.
            plot_image_with_cuboids(cuboid);
        end
    end
    if (save_plot_to_img)
        edge2d_img_title = sprintf('Frame %d,  best objects',frame_index);
        saved_2d_img_name = [saved_img_result_overview_dir sprintf('%04d_best_objects',frame_index)];
        save_figure_to_img(figure_2d_id, edge2d_img_title, saved_2d_img_name,false);
    end
end