A ROS package for bidirectional conversion between LLA (Latitude, Longitude, Altitude) and UTM (X, Y, Z in meters) coordinate systems. Also includes a GPS/heading parser node for reading NMEA GGA and UNIHEADINGA sentences from a serial device.
# Requirement
GeographicLib (test on version 1.34)
https://geographiclib.sourceforge.io/

# Download and build
```
mkdir -p catkin_ws/src
cd catkin_ws/src 
git clone https://github.com/Arcane-01/ros_gps_utm_converter.git
cd ..
catkin build
```

# Use
open a terminal and run roslaunch
```
roslaunch utm_lla coordinate_convertion.launch
```
Specify your UTM zone in the above launch file.  
if you want to convert LLA to UTM, publish your message to topic `/gps/fix`, ros message type `sensor_msgs::NavSatFix`. If you want to convert UTM to LLA, publish your message to topic `/utm/pose`, ros message type `geometry_msgs::PoseStamped`
