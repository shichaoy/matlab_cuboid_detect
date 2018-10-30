% file to filter and clean 2D object detections. There might be duplicated 2d bbox of the same object instance. remove one of them.
% can also be used to associate 2D bouding box with the ground truth 2D bbox

% This file doesn't use 3D cuboids!! Only based on 2d bounding box!!

clear;
close all;

dataset_name = 'cabinet';  % 'cabinet'
switch dataset_name
    case 'cabinet'  % one sample dataset
        data_proc_root_dir = 'path/to/TUM_RGBD/fr3_cabinet/';  % set up path here.
        yolo_det_thre = 0.15;
end


% important parameters
filtering_object_prob = 0.15;       % delete objects smaller than this
whether_remove_close_boundary_box = false;
whether_remove_overlapping_box = true;


data_yolo_objdec_dir = [data_proc_root_dir 'yolov2_obj_txts/'];  % this is the output of raw 2d object detections

saved_img_filter_match_2d_boxes_all_dir = [data_proc_root_dir 'filter_match_2d_boxes/'];   % saved image folder
saved_select_2dbox_match_mat_name = [data_proc_root_dir 'mats/select_2d_boxes_and_matches.mat'];  % saved mat name
if (~exist(saved_img_filter_match_2d_boxes_all_dir))    mkdir(saved_img_filter_match_2d_boxes_all_dir); end  % create folder if not having
if (~exist([data_proc_root_dir 'mats']))     mkdir([data_proc_root_dir 'mats']);  end

write_box_into_imgs = true; whether_plot_images=false; % true false
save_box_into_mat = false;

object_colors={'red','green','blue','cyan','magenta','yellow','red','green','blue','cyan'};


%% read yolo 2D object detection, remove some bad, overlapped boxes. all in 2D space.
whether_reselect_yolo_2d_boxes = true;
if (whether_reselect_yolo_2d_boxes)
    total_img_num = size(dir([data_proc_root_dir 'raw_imgs/' '*.jpg']),1);
    all_select_2d_bboxes = cell(total_img_num,1);
    all_select_2dboxes_classes = cell(total_img_num,1);    
    for frame_index = 0:total_img_num-1
        if (mod(frame_index,100)==0)
            disp(['Processing image ',num2str(frame_index)]);
        end

        %% read input images
        rgb_img = imread([data_proc_root_dir 'raw_imgs/' sprintf('%04d_rgb_raw.jpg',frame_index)]);
        im_width = size(rgb_img,2);  im_height = size(rgb_img,1);

        % read 2D object detection
        obj_bbox_structs = importdata([data_yolo_objdec_dir sprintf('%04d_yolo2_%.2f.txt',frame_index,yolo_det_thre)]);  % [x1 y1 x_width y_height prob]   x1 y1 is top-left coordinate
        
        if (size(obj_bbox_structs,1)>0)  % if found objects...
            obj_bbox_class_raw=obj_bbox_structs.textdata;  obj_bbox_coors_raw=obj_bbox_structs.data;obj_bbox_coors_raw(:,1:2)=obj_bbox_coors_raw(:,1:2)+1;  % +1 as c++ cnn index starts at 0

            if ( sum(sum(obj_bbox_coors_raw<=0))>0 )
                disp(['Finding negative coordinate.... ',num2str(frame_index)]);
            end
            
            % check object localtion not exceed image width and height
            x_exceed = (obj_bbox_coors_raw(:,1)+obj_bbox_coors_raw(:,3))>im_width;
            y_exceed = (obj_bbox_coors_raw(:,2)+obj_bbox_coors_raw(:,4))>im_height;
            if any(x_exceed)
                bad_x_ind = find(x_exceed);
                for jj=bad_x_ind' % row vec
                    fprintf('X exceeds max width at Frame %04d, line %d, exceed %d\n',frame_index, jj, obj_bbox_coors_raw(jj,1)+obj_bbox_coors_raw(jj,3)-im_width);    
                end
                obj_bbox_coors_raw(bad_x_ind,3) = im_width - obj_bbox_coors_raw(bad_x_ind,1);
            end
            if any(y_exceed)
                bad_y_ind = find(y_exceed);
                for jj=bad_y_ind'
                    fprintf('Y exceeds max height at Frame %04d, line %d, exceed %d\n',frame_index, jj, obj_bbox_coors_raw(jj,2)+obj_bbox_coors_raw(jj,4)-im_height);    
                end
                obj_bbox_coors_raw(bad_y_ind,4) = im_height - obj_bbox_coors_raw(bad_y_ind,2);                    
            end
        else
            obj_bbox_class_raw=cell(0,1);obj_bbox_coors_raw=zeros(0,5);
        end

        if(whether_plot_images)        
            figure(10);clf;imshow(rgb_img);title('Raw Detected 2D object');hold on;
            if (size(obj_bbox_structs,1)>0)  % if found objects...
                for id=1:size(obj_bbox_coors_raw,1)
                    rectangle('Position',obj_bbox_coors_raw(id,1:4),'EdgeColor',object_colors{id},'LineWidth',2);
                    text(obj_bbox_coors_raw(id,1)+5,obj_bbox_coors_raw(id,2)-10, sprintf('%s---%.2f',obj_bbox_class_raw{id},obj_bbox_coors_raw(id,5)),'Color',object_colors{id})
        %             pause();
                end
                pause(0.5);
            end
        end
        obj_bbox_coors = obj_bbox_coors_raw;
        obj_bbox_class = obj_bbox_class_raw;
        
        
        % Reject some object bounding box.
        obj_bbox_coors_s1 = obj_bbox_coors_raw(obj_bbox_coors_raw(:,5)>filtering_object_prob,:); % delete object with low prob
        obj_bbox_class_s1 = obj_bbox_class_raw(obj_bbox_coors_raw(:,5)>filtering_object_prob,:);
        if (whether_remove_close_boundary_box)  % if it is left or right close to boundary, delete it
            close_bound_ids = [];
            close_bound_margin = 10;
            for i=1:size(obj_bbox_coors_s1,1)
                obj_rect = obj_bbox_coors_s1(i,1:4);
                if ((obj_rect(1)<close_bound_margin) || (obj_rect(1)+obj_rect(3)>im_width-close_bound_margin) || (obj_rect(2)<close_bound_margin) || (obj_rect(2)+obj_rect(4)>im_height-close_bound_margin) )
                    close_bound_ids = [close_bound_ids;i];
                end
            end
            obj_bbox_coors_s1(close_bound_ids,:)=[];
            obj_bbox_class_s1(close_bound_ids,:)=[];
        end
        
        obj_bbox_coors = obj_bbox_coors_s1;
        obj_bbox_class = obj_bbox_class_s1;
        
        % most important part. if two bbox overlapping is large, remove one of them, with low prob. if prob is similar, remove small area one.
        if ( whether_remove_overlapping_box )
            can_force_merge = 1;
            counter = 0;
            delete_ids=[];
            while ( (can_force_merge==1) && (counter<500))
                counter=counter+1;
                can_force_merge=0;    
                for id1=1:(size(obj_bbox_coors_s1,1)-1)
                    for id2=(id1+1):size(obj_bbox_coors_s1,1)
                        [overlap_1,overlap_2]= bbox_overlap_ratio(obj_bbox_coors_s1(id1,1:4),obj_bbox_coors_s1(id2,1:4));        
                        if (overlap_1>0.5 || overlap_2>0.5)  % large overlap , need to delete one.
                            if (obj_bbox_coors_s1(id1,5)-obj_bbox_coors_s1(id2,5)>0.20)  % obj1 prob is better                
                               delete_ids=id2;
                            elseif (obj_bbox_coors_s1(id2,5)-obj_bbox_coors_s1(id1,5)>0.20)  % obj2 prob is better                
                               delete_ids=id1;
                            else   % if prob is similar. remove the one that being overlapped more...
                                if overlap_1>overlap_2
                                    delete_ids=id1;
                                else                    
                                    delete_ids=id2;
                                end
                            end
                            obj_bbox_coors_s1(delete_ids,:)=[];
                            obj_bbox_class_s1(delete_ids,:)=[];
                            can_force_merge=1;
                            break;
                        end         
                    end
                    if (can_force_merge==1)
                        break;
                    end
                end
            end

            if(whether_plot_images)
                figure(15);clf;imshow(rgb_img);title('Selected non-overlap 2D object');hold on;    
                if (size(obj_bbox_coors_s1,1)>0)    
                    for id=1:size(obj_bbox_coors_s1,1)
                        rectangle('Position',obj_bbox_coors_s1(id,1:4),'EdgeColor',object_colors{id},'LineWidth',2);
                        text(obj_bbox_coors_s1(id,1)+5,obj_bbox_coors_s1(id,2)-10, sprintf('%s---%.2f',obj_bbox_class_s1{id},obj_bbox_coors_s1(id,5)),'Color',object_colors{id})
                    end
                    pause(0.5);
                end
            end

            obj_bbox_coors = obj_bbox_coors_s1;
            obj_bbox_class = obj_bbox_class_s1;

            if (size(obj_bbox_coors,1)==0)    
                fprintf('Not found valid object of frame %d \n',frame_index);
            end
        end
        if (write_box_into_imgs)  % don't need plot, directly generate images
            rgb_img_cp=rgb_img;
            if (size(obj_bbox_coors,1)>0)        
                for id=1:size(obj_bbox_coors,1)
                    box_position = int32(obj_bbox_coors(id,1:4));rgb_img_cp = insertShape(rgb_img_cp, 'Rectangle',box_position,'LineWidth',3,'Color',get_id_color(id));
                end
            end
            imwrite(rgb_img_cp,[saved_img_filter_match_2d_boxes_all_dir sprintf('%04d',frame_index) '_yolo_2dobj_select.jpg'])
        end        

        all_select_2d_bboxes{frame_index+1} = obj_bbox_coors;
        all_select_2dboxes_classes{frame_index+1} = obj_bbox_class;
%         if(whether_plot_images)
%             pause();
%         end
    end
    if (save_box_into_mat)
        save(saved_select_2dbox_match_mat_name,'all_select_2d_bboxes','all_select_2dboxes_classes')
    end
else
    load(saved_select_2dbox_match_mat_name);
end


% save the selected boxes to txt.  matlab coordinate index. not raw c++ index
if (0)
    %load(saved_select_2dbox_match_mat_name);
    saved_img_filter_match_2d_boxes_txts_all_dir = [data_proc_root_dir 'mats/filter_match_2d_boxes_txts/'];    % save txt folder
    if (~exist(saved_img_filter_match_2d_boxes_txts_all_dir))
        mkdir(saved_img_filter_match_2d_boxes_txts_all_dir);
    end
    for frame_index=0:length(all_select_2d_bboxes)-1
        if (mod(frame_index,10)==0)
            disp(['Processing image ',num2str(frame_index)]);
        end
        obj_bbox_coors = all_select_2d_bboxes{frame_index+1};
        obj_bbox_class = all_select_2dboxes_classes{frame_index+1};

        save_selected_obj_txt_name = [saved_img_filter_match_2d_boxes_txts_all_dir  sprintf('%04d_yolo2_%.2f.txt',frame_index,yolo_det_thre)];
        fid_2 = fopen(save_selected_obj_txt_name, 'w');    
        for line_id = 1:size(obj_bbox_coors,1)    
            for j = 1:4
                fprintf(fid_2,'%d\t',obj_bbox_coors(line_id,j));
            end
            fprintf(fid_2,'%.2f\n',obj_bbox_coors(line_id,end));            
        end
        fclose(fid_2);        
    end
end








%% Another similar function as above.  Match selected 2D yolo bounding box with ground truth bounding box, based on overlapping ratio.
% need to prepare many things: ground truth objects. filtered predicted 2d bounding box.
whether_rematch_2d_boxes = false;
save_box_into_imgs = false; whether_plot_images = false; % true false
save_box_into_mat = false;
if (whether_rematch_2d_boxes)
    all_pred_matched_truths = cell(total_img_num,1);  % the row of each cell is the selected yolo object number
    frame_find_matches = zeros(total_img_num,1);
    for frame_index=0:(total_img_num-1)
        % close all;
        if mod(frame_index,100)==0
            disp(['Processing image ',num2str(frame_index)]);
        end

        % load ground truth boxes
        frame_calib_mat = frame_full_infos(frame_index+1);        
        if (strcmp(dataset_name,'kitti_object'))
            truth_object_num = length(frame_calib_mat.truth_objects);
        end

        if (truth_object_num==0)
        %     disp(['zero truth box ',num2str(frame_index)]); 
        end
        
        % load predicted object
        pred_objs = all_select_2d_bboxes{frame_index+1};

        rgb_img = imread(frame_calib_mat.rgbpath);
        if (whether_plot_images)  % plot truth bbox   sun rgbd data format.
            rgb_img = imread(frame_calib_mat.rgbpath);            
            figure(1); imshow(rgb_img); hold on; 
            for kk =1:length(frame_calib_mat.groundtruth3DBB_tight)
                rectangle('Position', [frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(1) frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(2) frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(3) frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(4)],'edgecolor','y');
                text(frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(1),frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D(2),frame_calib_mat.groundtruth3DBB_tight(kk).classname,'BackgroundColor','y')
            end
            pause(0.5);
            if (save_box_into_imgs)
                edge3d_img_title = sprintf('%d, truth 2D boxes',frame_index);
                saved_3d_img_name = [saved_img_filter_match_2d_boxes_all_dir sprintf('%04d_truth_box',frame_index)];
                save_figure_to_img(1, edge3d_img_title, saved_3d_img_name);
            end
        end

        if (whether_plot_images)  % plot predict bbox
            figure(15);imshow(rgb_img);title('Predicted 2D boxes');hold on;        
            for object_id = 1:size(pred_objs,1)  % for each yolo detected object
                rectangle('Position',pred_objs(object_id,1:4),'EdgeColor',object_colors{object_id},'LineWidth',2); % randomize image color?
            end
            pause(0.5);
            if (save_box_into_img)
                edge3d_img_title = sprintf('%d, pred 2D boxes',frame_index);
                saved_3d_img_name = [saved_img_filter_match_2d_boxes_all_dir sprintf('%04d_pred_box',frame_index)];
                save_figure_to_img(15, edge3d_img_title, saved_3d_img_name);
            end
        end

        % for each predicted object, 
        truth_used = zeros(truth_object_num,1);
        pred_matched_truth = ones(pred_object_num,2)*(-1);   % even if there is no 3D cuboid samples, give it -1
        iou_2d_thre = 0.7;  % 0.5  nearly the same
        for object_id = 1:pred_object_num  % for each yolo detected object
                yolo_2d_rect = pred_objs(object_id,1:4);

                overlapRatios = zeros(truth_object_num,1);
                for kk =1:truth_object_num
                    if (size(frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D,1)>0)
                        overlapRatios(kk) = bboxOverlapRatio(yolo_2d_rect,frame_calib_mat.groundtruth3DBB_tight(kk).gtBb2D);
                    end
                end
                [max_iou,max_id]=max(overlapRatios);
                if (max_iou>iou_2d_thre)
                    if (~truth_used(max_id))  % the truth object is not assigned to other cuboids
                        pred_matched_truth(object_id,:)=[max_id max_iou];  % start from 1
                        truth_used(max_id)=1;
                    else
        %                disp(['truth object used ',num2str(frame_index)]); 
                    end
                else
        %            disp(['max iou small ',num2str(frame_index),' IoU  ',num2str(max_iou)]);
                end        
        end
        all_pred_matched_truths{frame_index+1} = pred_matched_truth;

        % % if this is no matched pair
        if ~(any(pred_matched_truth(:,1)>0))
            disp(['No matching pair ',num2str(frame_index)]);
        else
            frame_find_matches(frame_index+1)=1;
        end

        % draw dotted rect on it (with probability), same color with the matched ones, then save the image, test to see each image.
        if (false)            
            if (frame_find_matches(frame_index+1)==1) % if has matches!!!
                rgb_img = imread(frame_calib_mat.rgbpath);
                rgb_img_cp = rgb_img;
                for object_id = 1:pred_object_num  % for each yolo detected object
                   if (pred_matched_truth(object_id,1)>0)                       
                       rgb_img_cp = insertShape(rgb_img_cp, 'Rectangle',pred_objs(object_id,1:4),'LineWidth',3,'Color',object_colors{mod(object_id,10)+1});
                       rgb_img_cp = insertShape(rgb_img_cp, 'Rectangle',frame_calib_mat.groundtruth3DBB_tight(pred_matched_truth(object_id,1)).gtBb2D,'LineWidth',1,'Color',object_colors{mod(object_id,10)+1});
                   end
                end
%                 pause(0.5);

                if (save_box_into_img)
                    % could also plot then save images, but slow. see previous example.
                    saved_3d_img_name = [saved_img_filter_match_2d_boxes_all_dir sprintf('%04d_%.2f_matched_box.jpg',frame_index,iou_2d_thre)];
                    imwrite(rgb_img_cp,saved_3d_img_name);
                end
            end
        end
    end
    if (save_box_into_mat)
        save(saved_select_2dbox_match_mat_name,'all_select_2d_bboxes','all_select_2dboxes_classes','all_pred_matched_truths','frame_find_matches')
    end
% else
%     load(saved_select_2dbox_match_mat_name);
end


% save as truth matches into txt
if (0)
    for frame_index=0:size(all_select_2dboxes,1)-1
        pred_truth_matches = all_pred_matched_truths{frame_index+1};
        gmu_2d_box_match_txt = [data_proc_root_dir 'mats/gmu_2dboxes_matches_txts/' sprintf('%04d_truth_matches.txt',frame_index)];
        dlmwrite(gmu_2d_box_match_txt,pred_truth_matches,'delimiter','\t','precision',3);
    end
end
