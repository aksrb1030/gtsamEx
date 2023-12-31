# gtsamEx

## Nonlinear factor graph optimization visualizer

* Optimization .g2o and control to iterlation in dynamic_reconfigure

## Dependency
<!-- [NoeticLink] :  -->

* ROS (tested in [Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) only)
```
sudo apt-get install ros-noetic-jsk-visualization
```

* gtsam
```
sudo apt-get install ros-noetic-gtsam
```

## Install
Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git  clone https://github.com/aksrb1030/gtsamEx
cd ..
catkin_make
```


## Run the pacakge
```
roslaunch gtsam_ex run_gtsam.launch
```

## Visualizer

### dynamic_reconfigure
<img src="./img/dynamic_reconfigure.png">

### sphere.g2o(before)
<img src="./img/sphere_before.png">

### sphere.g2o(after)
<img src="./img/sphere_after.png">

### parking-garage_after.g2o(before)
<img src="./img/parking-garage_before.png">

### parking-garage_after.g2o(after)
<img src="./img/parking-garage_after.png">