# Efficient Online Segmentation of Ground&Wall Points

## 1. Introduction 
This is a **light-weight & efficient** online segmentation algorithm for segmenting **ground points and wall points** out of an arbitrary LiDAR point cloud. 

This algorithm is inspired by *"M. Himmelsbach, F. v. Hundelshausen and H. -. Wuensche, **Fast segmentation of 3D point clouds for ground vehicles**, 2010 IEEE Intelligent Vehicles Symposium, 2010"*, an original implementation can be accessed here [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation).

See the picture below for a first look, where yellow lines(also magenta points) indicate ground points, and green lines(also green points) indicate wall points.
![result](pics/result04.png)

## 2. Results & Videos 
`coming soon ...`  
A test video can be seen on my personal Zhihu homepage, click [here](https://zhuanlan.zhihu.com/p/508961457).

## 3. Algorithm Detail & Implementation
`coming soon ...`  
A chinese version can be seen on my personal Zhihu homepage, click [here](https://zhuanlan.zhihu.com/p/508961457).

## 4. Dependencies 
This project is also light-weight in dependencies, only **PCL & OpenCV** are required, you can easily install these two by `apt-get install`. Note that PCL is essential while OpenCV is only for visualization of range image. A developer should take no trouble to remove OpenCV dependency with just a little bit modification on original codes.

## 5. Compile & Run
Once you have solved the dependencies problem, just execute `catkin_make` and `roslaunch efficient_online_segmentation efficient_online_segmentation.launch`. See the *.launch* file for detail.

By default, **VLP-16 LiDAR scanner** or any other 16-rings LiDAR are supported. If you want to see how **32/64/128 LiDAR scanners** work, just modify LiDAR related parameters in `sgmtt_utility.h/SegmentationParams`.

## 6. Others
...
