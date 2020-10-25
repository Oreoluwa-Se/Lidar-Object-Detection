# Lidar-Object-Detection
* This repo is for the Lidar object detection project from the [Udacity Sensor Fusion Engineer](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313).
* The aim of the project is to perform object detection and segmentation using real pcd data provided by udacity.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal.

# Project Directory:
The project directory should be similar to:
```
├── build                           # build files for program
│   ├── ...
│── gifs                            # stores video output from project                   
│   ├── filtered_points.gifs
│   ├── bounding_box.gifs
│   ├── rotated_bounding.gifs
│   ├── segmented.gifs
├── src                     
│   ├── helper                      # helper functions for program
│   │   ├── ransac3d.h              # utility function which used to segment data points
│   ├── render                      # simulates the PI controller
│   │   ├── box.h                   # stores structs for box definition
│   │   ├── render.cpp              # function used for image display
│   │   ├── render.h                # header file for rendering images   
│   ├── sensors                     # stores lidar data
│   │   ├── data                    # stores pcd data
│   ├── environment.cpp             # main program for insta
│   ├── processPointClouds.cpp      # contains functions for processing point clouds
│   ├── processPointClouds.h        # header file for point cloud processing
├── CMakeLists
├── Readme.md

```