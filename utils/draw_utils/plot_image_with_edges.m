function plot_image_with_edges(figure_num, rgb_img, edges, title_string, edge_color, whether_new_imiage, pause_debug)
% draw rgb image and edges. edges is n*4 matrix  [x1 y1 x2 y2]

    figure(figure_num);
    if (whether_new_imiage)  % if this is not new image, previous image should already be drawn!
        imshow(rgb_img);
    end
    if (size(title_string,2)>0)
        title(title_string);
    end
    hold on;
        
    if (nargin<7)
        pause_debug=false;
    end
    for i=1:size(edges,1)        
        if (pause_debug)
            plot([edges(i,1) edges(i,3)],[edges(i,2) edges(i,4)],get_id_color(i),'Linewidth',2.5);
            pause();
        else
            plot([edges(i,1) edges(i,3)],[edges(i,2) edges(i,4)],edge_color,'Linewidth',2.5);
        end
    end
    
    pause(0.5);
end