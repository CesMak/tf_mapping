# tf_mapping

1) create a map:


2) navigate in this map:

Fixed marker moving camera!

Stelle so auf, dass zuerst nur Marker_Id = 0 footprint sieht. Dadurch werden folgende Marker erstellt:
* Dort wo die Kamera aktuell ist: marker_globe_0
* Dort wo man den Ursprung haben möchte: marker_globe_1 = *footprint of origin* -> set fixed frame to marker_globe_1 (Die Position und Orientierung unter tf in RVIZ von Bsp. marker_globe_2 ist dann relativ zu marker_globe_1 (die dort angegebene relative position ist bezüglich des world frames))
* Bewege dann Kamera so dass Marker_Id = 0 und Marker_Id = 1 auf einem Bild zu sehen sind dadurch wird dann marker_globe_2 erstellt.
* Fahre dann so weiter fort um map aufzubauen.
* starte python skript um map abzuspeichern
* starte roslaunch die automatisch fake_marker published.
* diese fake_marker können dann zur navigation genutzt werden.
 

// rostopic echo /scan |grep frame_id
// rosrun tf tf_monitor /marker_globe_1 /marker_globe_2
// rosrun tf tf_echo /marker_globe_1 /marker_globe_2
- Translation: [-0.043, -0.506, 0.181]
- Rotation: in Quaternion [0.016, 0.690, 0.722, 0.056]

see also: http://wiki.ros.org/navigation


TODO 
![aruco_detector_example](https://github.com/CesMak/aruco_detector_ocv/blob/master/data/rviz_example.png)

The aruco_detector_osv (aruco_detector_opencv) uses the #include <opencv2/aruco.hpp> library instead of using #include <aruco/aruco.h> (ros-kinetic-aruco). This package was tested on ubuntu 16.04, ROS Kinetic with a Logitech C920 camera. 

With this package you are able to:

* detect position and orientation of an [aruco marker](http://chev.me/arucogen/) relatively to the camera. The corresponding **tf** is published.
* a certainty parameter of how robust the arcuco marker is detected. 
* a result image with the detected markers highlighted is published.

Please calibrate your camera first using: [camera_calibration](http://wiki.ros.org/camera_calibration).

If you get (when using the camera)

``` 
[ERROR] [1551630633.628039127]: Cannot identify '/dev/video0': 2, No such file or directory
```

do 

``` 
sudo modprobe uvcvideo
``` 

See all adjustments in the **aruco_detector_osv/launch/detector.launch** 

## Installation
install dependencies, git, use ros kinetic, ubuntu 16.04.

``` 
cd ~/Desktop
source /opt/ros/kinetic/setup.bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
git clone git@github.com:CesMak/aruco_detector_ocv.git (takes some time due to included bag to test this package)
cd ..
catkin init -w .
catkin build
source devel/setup.bash
roscd roscd aruco_detector_ocv/data/
rosbag decompress 640x480_logitech_aruco3_compressed.orig.bag 
roslaunch aruco_detector_ocv detector.launch 
```


## Launch

``` 
roslaunch aruco_detector_ocv detector.launch 
``` 


## Dependencies:
cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport std_msgs

## Further adjustements

There are many opencv parameters that can be adjusted internally to reduce the effect of lightening conditions.
See [opencv_tutorial](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html)

In order to adjust camera options use **scripts/marker_filter.py**

## License BSD
If you want to use this package please contact: [me](https://simact.de/about_me).

