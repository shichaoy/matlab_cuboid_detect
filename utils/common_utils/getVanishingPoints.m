function [vp_1,vp_2,vp_3] = getVanishingPoints(Kalib, invR, yaw)
% word frame is on ground (xy on ground, z vertical up): assume object lies on ground, with zero roll, pitch.  invR is world to camera rotation. yaw is object

    vp_1 = (homo_to_real_coord(Kalib*invR*[cos(yaw) sin(yaw) 0]'))';  % for object x axis
    vp_2 = (homo_to_real_coord(Kalib*invR*[-sin(yaw) cos(yaw) 0]'))'; % for object y axis
    vp_3 = (homo_to_real_coord(Kalib*invR*[0 0 1]'))'; % for object z axis
