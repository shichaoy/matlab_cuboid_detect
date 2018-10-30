# Matlab cuboid detect

Matlab code to detect cuboid object. There are two main parts: cuboid proposal generation, and proposal scoring.

**NOTE** We also provide a C++ version of cuboid detection and multi-view object SLAM. Please see [cube_slam](https://github.com/shichaoy/cube_slam).


**Authors:** [Shichao Yang](http://www.frc.ri.cmu.edu/~syang/)

**Related Paper:**

* **CubeSLAM: Monocular 3D Object Detection and SLAM without Prior Models**, Arxiv 2018, S. Yang, S. Scherer  [**PDF**](https://arxiv.org/abs/1806.00557)

<br>



## How to run:
```git clone git@github.com:shichaoy/matlab_cuboid_detect.git``` 

then open Matlab

```bash
cd matlab_cuboid_detect
init_setup.m
detect_cuboid.m
```



### Notes

1. **Overview:** See ```illustrations.pdf``` for corner indexing in generating proposals and 3D coordinate system.

2. ```data/``` folder contains some preprocessing results. Edge detection is from this ros package [line_lbd](https://github.com/shichaoy/pop_up_slam/tree/master/line_lbd). The mat contains the 2D object bounding boxes. We use Yolo to detect 2D objects. Other similar methods can also be used. ```preprocessing/2D_object_detect``` is our prediction code to save images and txts. Sometimes there might be overlapping box of the same object instance. We need to filter and clean some detections. ```filter_match_2d_boxes.m``` for more details.


