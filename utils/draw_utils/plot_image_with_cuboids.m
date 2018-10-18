function plot_image_with_cuboids(cuboid, simple_color)
% plot cuboid onto existing figure. an image should already be plotted
% cuboid struct should contain necessary fields

if nargin<2
    simple_color = false;
end

hold on;

[edge_markers,line_marker_type] = get_cuboid_draw_edge_markers(cuboid.box_config_type, true);  % each row: edge_start_id,edge_end_id,edge_marker_type_id

if (simple_color)
    line_marker_type={'r','g','r','g','r','g'};
%     line_withds = [2 1];   % visible/hidden
end

for edge_id=1:size(edge_markers,1)
    plot(cuboid.box_corners_2d(1,edge_markers(edge_id,1:2)),cuboid.box_corners_2d(2,edge_markers(edge_id,1:2)),line_marker_type{edge_markers(edge_id,3)},'Linewidth',3.0);
end