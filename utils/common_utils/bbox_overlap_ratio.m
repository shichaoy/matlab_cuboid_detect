function [overlap_1,overlap_2]= bbox_overlap_ratio(bboxA,bboxB)
% overlapping ratio of two bbox. each is 1*4 [x y width height]   compute: union/Area(1)   union/Area(2)

    intersectionArea = rectint(bboxA,bboxB);
    overlap_1 = intersectionArea/bboxA(3)/bboxA(4);
    overlap_2 = intersectionArea/bboxB(3)/bboxB(4);