# Vision Programming Challenge
## Table of Contents
1. [Challenge Description](#challenge-description)
2. [Solution for Task Basic](#solution-for-task-basic)
2. [Solution for Task Vision](#solution-for-task-vision)
3. [References](#references)

## Challenge Description
### Introduction
Thank you for agreeing to spend your time solving this programming challenge in the vision domain for us. We will evaluate your solution based on accuracy, code quality, and how clearly you describe your technique.

### Problem Statement
This task attempts to gain a better understanding of your knowledge of computer vision as well as your programming capability. Edge detection is a preliminary but important step in computer vision. The goal of this challenge is to detect the edges of a checkboard and highlight them with a green line superimposed on the image.

Different sample images are provided to test your code with. The ideal solution will be modular, object-oriented, and able to deal with different image sizes and different numbers of squares and their respective sizes. It should also be robust to noise and rotation.

The different filters used should be implemented in a modular and parameterizable form. Popular image processing libraries like OpenCV, which are open source, can be used. It is ideally expected that the candidate solves the problem using the C++ language. Python can be used, but Matlab is not permitted.

### Preliminary Requirements
1. Install ROS Noetic
2. Create a catkin workspace
3. Clone this repository into the src/ directory of your workspace
4. Ready to work on your code and build. 

    See ROS Tutorials: http://wiki.ros.org/ROS/Tutorials 

5. Download bagfiles from a directory that will be shared to you.

6. To launch the robot description and bag file:
    ```
    # Terminal 1
    roscore

    # Terminal 2
    rosparam set /use_sim_time true
    source <path to catkin workspace>/devel/setup.bash
    roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false

    # Terminal 3
    rosbag play --clock -l <path to bagfile>
    ```
### Tasks and results
#### 1. Basic 
For C++, the EdgeDetector.hpp and EdgeDetector.cpp files should be filled in. If any libraries are required, add them to the CMakeLists.txt. For Python, the edge_detector.py file should be filled in. Instructions must also be provided to download and install any required Python packages.

It should be possible to provide different images as input to your code and show the output as the image with the edges detected as green lines. Also, provide a Readme.md file detailing instructions on installation, implementation steps, concepts used, and possible improvements. 

#### 2. Vision_ROS: 
Provide ROS .srv and .msg files required to create a ROS service for edge detection. Give example usage of this service with a client to detect edges for image files in a directory.

Additionally, detect edges for the images subscribed from an image topic that is available when the given ROS bag (.bag) file is played. The input image and detected edges should be visualised on RViz.

Convert the detected edge pixels from pixel coordinates (u, v) to 3D data (x, y, z) using the depth images and camera parameters also available in the .bag file. Publish the 3D data to a ROS topic (suggestion: of type sensor_msgs/PointCloud), with a topic name edge_points.

#### 3. Robot_ROS: 
Extend the code further by visualizing the 3D edge points for each frame as RViz markers together with the visualization of a robot. You can use the robot URDF model provided to visualize the robot and multiple frames of reference. Please provide a video of the markers and the robot on Rviz for a duration of one loop of the given .bag file as part of the submission.

#### 4. Advanced: 
Do all the above tasks.

## Solution for Task Basic

This section descibes the solution to the basic task.

### Installation
1. Clone this repository into your catkin workspace.
2. [Install OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
3. Run catkin_make from catkin workspace.

### Usage
```
Detects edges in a specified image. Implements different methods for edge detection. Currently the canny edge detector and an addaptive version of the canny edge detector (auto canny) are implemented. For preprocessing, a bliateral filter as described in https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html can be used.
Usage: EdgeDetection [params] path edge_detector 

        -?, -h, --help, --usage (value:true)
                Shows the help message.
        --aperture_size (value:3)
                Only used for canny edge detectors. Aperture size of the Sobel operator for canny edge detectors.
        --bf_diameter (value:15)
                Only used if 'no_bf' is not specified. Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from bf_sigma_space.
        --bf_sigma_color (value:200)
                Only used if 'no_bf' is not specified. Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see bf_sigma_space) will be mixed together, resulting in larger areas of semi-equal color.
        --bf_sigma_space (value:200)
                Only used if 'no_bf' is not specified. Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see bf_sigma_color ). When bf_diameter>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace.
        --l2, --l2_gradient
                Only used for canny edge detectors. If true, the L2-norm is used to compute gradients of the image. Otherwise, the L1-norm is used.
        --no_bf, --no_bilateral_filter
                If not specified, the image is preprocessed with a bilateral filter, parametrized by the values specified for  bf_diameter, bf_sigma_color and bf_sigma_space.
        --sigma (value:0.33)
                Only used if edge_detector is 'auto_canny'. The sigma value for computing the lower and upper hysteresis threshold.
        --threshold1 (value:200)
                Only used if edge_detector is 'canny'. Determines the lower hysteresis theshold.
        --threshold2 (value:225)
                Only used if edge_detector is 'canny'. Determines the upper hysteresis theshold.

        path (value:./src/edge_detection_ros/edge_detection/data/index.jpeg)
                The path to the image of interest.
        edge_detector (value:canny_auto)
                The type of edge detector to used. Options are: canny_auto, canny. Depending on thos value, the following parameters are determined.
```
### Concepts Used
In literature different methods for edge detection are presented [[1]](#1). These can be divided into traditional methods and methods based on Machine Learning. A simple traditional method with an [open source implementation in OpenCV](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html) is the Canny edge detector [[2]](#2). As a first approach this method is chosen. Various versions of this classic algorithm exist, e.g. an [adaptive extension](https://pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/), which computes the hysteresis thresholds for the Canny algorithm. To make the Canny algorithm more robust towards noise, additional filters can be used in preprocessing. One of theses filtes is e.g. the Bilateral Filter [[3]](#3). This filter is better suited for edge detection, becasue it keeps the edges sharper and easily accesible, because it is [implemented in OpenCV](https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed), too.

### Implementation Details
In this task two different versions of the Canny edge detector are implemented. The original canny edge detector (`CannyEdgeDetector`) and an adaptive version (`AutoCannyEdgeDetector`). Each version is implemented in a separate class, inheriting from the class `BaseEdgeDetector`. This abstract base class requires the implementation of the the method `detectEdges` and provides common functionalities like preprocessing and the superimposing of images. This abstraction allows adding more methods in the future. The implementation of the original Canny algorihtm uses the OpenCV implementation.

Preprocessing is implemented in `BaseEdgeDetector`. It contains a vector of potentially multiple filters. Preprocessing filters are abstracted in the base class `Filter`. A class inheriting from `Filter` is e.g. `BilateralFilter`, implementing the Bilateral filter based on the OpenCV implementation. This additionaly abstraction allows adding more filters in the future, if needed.

### Possible Improvements
 - Tune default parameters 
 - Add additional edge detection methods, e.g. ML-based methods


## Solution for Task Vision
Note that this task is currently not finished.
### Usage
#### For the edge detection service
```
rosrun edge_detection edge_detection_service <Optional parameters>
```
Optional parameters are all CLI-parameters from the basic task except for `path`.

#### For the edge detection client
The client can be used in two modes. The first mode is used for testing and processes a path. This mode can be run by calling:
```
rosrun edge_detection edge_detection_service --path=<path>
```
The second mode mode can detect images from the camera, if the bag file is running. For this mode the path argument can be obmitted. This mode is not fully complete and currently only displays the edges in a window without transforming them to a point cloud.

## References
<a id="1">[1]</a>
Sun, R., Lei, T., Chen, Q., Wang, Z., Du, X., Zhao, W., & Nandi, A. K. (2022). Survey of Image Edge Detection. Frontiers in Signal Processing, 2. doi:10.3389/frsip.2022.826967

<a id="2">[2]</a>
Canny, J. (1986). A computational approach to edge detection. IEEE Transactions on pattern analysis and machine intelligence, (6), 679-698.

<a id="3">[3]</a>
Banterle, F.; Corsini, M.; Cignoni, P.; Scopigno, R. (2011). "A Low-Memory, Straightforward and Fast Bilateral Filter Through Subsampling in Spatial Domain". Computer Graphics Forum. 31 (1): 19–32. doi:10.1111/j.1467-8659.2011.02078.x
