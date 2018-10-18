# cuboid_detect

Matlab code to detect cuboid object. There are two main parts: cuboid proposal generation, and proposal scoring.

**Authors:** [Shichao Yang](http://www.frc.ri.cmu.edu/~syang/)

**Related Paper:**

* **CubeSLAM: Monocular 3D Object Detection and SLAM without Prior Models**, Arxiv 2018, S. Yang, S. Scherer  [**PDF**](https://arxiv.org/abs/1806.00557)

<br>



### Preprocessing

1. **Overview:** See ```illustrations.pdf``` for corner indexing in generating proposals and 3D coordinate system.

2. ```data``` folder contains some preprocessing results. We use yolo for 2D object detector. Edge detection is from this ros package [line_lbd](https://github.com/shichaoy/pop_up_slam/tree/master/line_lbd). Other similar methods can also be used.



## How to run:
```git clone git@github.com:shichaoy/cuboid_detect.git``` 

then open Matlab

```bash
cd cuboid_detect
init_setup.m
detect_cuboid.m
```
