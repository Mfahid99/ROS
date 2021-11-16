sudo i2cdetect -r -y 1
SDA > pins 3 & 27
SCL pins 5 & 28
#saving a rosbag file for map
rosbag record -O my_bagfile_1.bag /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu /camera/imu_info /tf_static
#replay rosbag map log
roscore >/dev/null 2>&1 &
rosparam set use_sim_time true
rosbag play my_bagfile_1.bag --clock
roslaunch realsense2_camera opensource_tracking.launch offline:=true
#c++ object of interest localize
https://github.com/twMr7/rscvdnn/blob/1dd3fc9453259a7c8dc7542f7738dc2c00c50b08/rscvdnn/MainWindow.cpp
#obstacle avoidance
https://support.intelrealsense.com/hc/en-us/community/posts/1500001395622-D435I-sample-code-for-obstacle-avoidance
#depth filtering for collision avoidance
https://dev.intelrealsense.com/docs/opencv-wrapper?_ga=2.34903585.1218669506.1636996045-404737928.1636996045#section-4-depth-filtering-for-collision-avoidance
https://www.qwrt.de/pdf/Depth_Map_Improvements_for_Stereo_based_Depth_Cameras_on_Drones.pdf
#example object avoidance 
https://github.com/Onlee97/Object-Detection-and-Avoidance-with-Intel-Realsense
