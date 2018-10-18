function [corners_3d_world] = compute3D_BoxCorner(object)
% output is 3*8, face_idx is fixed, depending on how we define front and back face.
%   8 ————5
%  /|    /|
% 7 4---6-1
% |/    |/ 
% 3 ————2
% cuboid frame definition: x y at bottom center. x rightward, y innerward, z upward.
% this cuboid frame and indexing/ordering is unified across all configurations!


    Tr = similarityTransformation(object); % [R*s t';0 0 0 1]
    
    % 3D bounding box corners
    x_corners = [1, 1, -1, -1, 1, 1, -1, -1];  % [-1 1] represent the boundary because object scale is half height/width/lenght!
    y_corners = [1, -1, -1, 1, 1, -1, -1, 1];
    z_corners = [-1, -1, -1, -1, 1, 1, 1, 1];
    corners_body=[x_corners;y_corners;z_corners];
    corners_3d_world=homo_to_real_coord(Tr*real_to_homo_coord(corners_body));
    
end