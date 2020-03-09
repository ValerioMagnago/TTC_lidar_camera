# SFND 3D Object Tracking

This repository reports the final project of the camera module of the Udacity Sensor Fusion nanodegree program. 
The project leverage the knowledge on keypoint detectors, descriptors, and methods to match them between successive images that have been studied in the module.
Nevertheless the YOLO neural network in the OpenCv framework is used to detect car and truck in the image sequence.
The detection is used to associate regions in a camera image with Lidar points in 3D space. In the figure below the program schematic is reported.

<img src="images/course_code_structure.png" width="779" height="414" />

With reference to the picture the main step of the program are:
1. **LOAD IMAGE INTO BUFFER**: for each timestamp at most 2 images are loaded into a circular buffer in order to limit the memory usage
2. **DETECT & CLASSIFY OBJECTS**: the standard pre-trained weights of yolo were used to detect cars and truck in the images sequence
3. **CROP LIDAR POINTS**: the pointcloud is cropped using a simple filter on the x, y, z coordinate value of the points to keep only point on the car in front of the ego-vehicle. 
4. **CLUSTER LIDAR POINT CLOUD**: knowing the rototranslation to express the pointcloud in the camera frame is it possible to match the camera detection with lidar information and cluster the car inside the pointcloud.
5. **DETECT KEYPOINTS**: the first step in order to compute the TTC from the camera image is to detect keypoints.
6. **EXTRACT DESCRIPTORS**: in this step at each keypoint is associated a descriptor.
7. **MATCH KEYPOINT DESCRIPTORS**: the descriptor of the actual and previous images are matched and correspondence between the 2 images are found.
8. **TRACK 3D OBJECT BOUNDING BOXES**: to have a coherent identification of the bounding boxes, correspondence between actual and previous bounding boxes are evaluated.
9. **TTC computation**: time to collision is computed in parallel and independently on lidar clustered information and on camera clustered information.


## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.