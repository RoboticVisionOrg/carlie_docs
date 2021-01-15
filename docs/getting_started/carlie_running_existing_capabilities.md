---
sort: 5
---

# Running Existing Demos and Capabilities on Carlie
There exists several pre-existing demos and capabilities for Carlie, including:

- [AR Tag Following Demo](#ar-tag-following-demo)
- [HSV Blob Following Demo](#hsv-blob-following-demo)
- [Vision-based Teach and Repeat](#vision-based-teach-and-repeat)
- [LiDAR Mapping](#lidar-mapping)

<!--
- [LiDAR Mapping](https://github.com/RoboticVisionOrg/carlie_apps/tree/master/gmapping_app)
- [LiDAR Localisation](https://github.com/RoboticVisionOrg/carlie_apps/tree/master/localisation_app) 
-->

Most of these can be found within the [carlie_apps](https://github.com/RoboticVisionOrg/carlie_apps) package and are launchable via a web interface. To launch one of the existing capabilities via the web interface, if available, perform the following:

1. Turn on Carlie and determine the IP address.
2. On another computer, on the same network as Carlie, open up an internet browser and enter Carlie's IP address into the URL.
3. Start the demo you desire.


## AR Tag Following Demo
The AR Tag Following Demo allows Carlie to follow a AprilTag. This demo utilises the [AprilTag ROS](https://wiki.ros.org/apriltag_ros) package. To run the AR Tag Following Demo you will require an AR Tag from the 36h11 with an ID of 5 ([PDF here](assets/apriltag_36h11_5.pdf)). The edge of the tag should be 0.161 metres, however it is fine if this is not the case. This demo can be run through the web interface or via the command line. To run the demo via the command line execute the following within a terminal (if you are SSH'ed into Carlie make sure you have an X server running):

    roslaunch carlie_ar_tag_following_demo demo.launch image_view_on:=true

## HSV Blob Following Demo
The HSV Blob Following Demo allows the user to perform colour and size segmentation to identify an object that Carlie should then follow. This demo can be run through the web interface or via the command line. To run this demo via the command line execute the following in a terminal (if you are SSH'ed into Carlie make sure you have an X server running):

    roslaunch carlie_hsv_blob_following_demo demo.launch

## Vision-based Teach and Repeat
The Vision-based Teach and Repeat allows the user to drive Carlie along a path (the teach phase), Carlie can then be returned to the starting point and can then autonomously navigate the path (repeat phase). To do this Carlie records data during the teach phase, in this case images, and then during the repeat phase attempts to match the current image to a given image recorded during the teach phase. This demonstration can only be run via the command line. The teach and repeat framework will perform the following high-level steps:

1. Start up the D435 RGB-D camera and check it is working
2. Run the teach phase. This will save images and odometry data
3. Preprocess the teach phase images ready for the repeat phase
4. Run the repeat phase and hope for the best

Details for each step are below. However, before running these you will need to clone the ROS package and perform a catkin_make:

    cd ~/catkin_ws/src/
    git clone https://github.com/RoboticVisionOrg/carlie_teach_and_repeat
    cd ~/catkin_ws
    catkin_make
    rospack profile


### Step 1: Start the Camera
We will now run the teach and repeat example. To start we will get the D435 RGB-D Realsense camera up and running. To start the camera run the following commands:

    roslaunch carlie_sensors carlie_sensors.launch lidar_enabled:=false pose_camera_enabled:=false

Sometimes the realsense cameras do not seem to start up correctly. It is something to do with their internal hardware and software. We are trying to fix this, but currently the only solution is to unplug and replug in the camera that isn't working or `ctrl+c` the command and relaunch. To check to see if the D435 RGB-D camera is working (the bottom camera), run the following in another terminal:

    rostopic echo --noarr /rgbd_camera/color/image_rect_color

If data/messages start appearing in your terminal, sometimes it takes 1-2 seconds, then the RGB-D camera is good to go.

### Step 2: Teach Phase
The first step is to run the teach phase. Place Carlie on the floor, if it isn't already, and make sure you can teleop Carlie using the Logitech Gamepad (see [Driving Carlie](../getting_started/starting_up_carlie#driving-carlie)). You may also wish to roughly mark the starting position (e.g. place a post-it note next to a wheel). Then run the following command:

    roslaunch carlie_teach_and_repeat teach.launch

Wait until the terminal says `Press B on the Gamepad to Start Recording`. Press B on the Gamepad and start driving. Carlie will save an image approximately every 0.25m as well as the odometry between frames. Drive a simple route that does not have tight corners, and try to keep away from obstacles. The starting and ending points of the route do not need to be the same. Press B once you have driven your route to stop recording data, this will terminate the node. The images and the odometry data will be saved in the directory `/home/nvidia/route_1`. **Note**: the `dataset.txt` file will not show anything until the recording is completed (i.e. B is pressed for a second time). 

Sometimes image data does not get through the ROS message pipeline, or is corrupted. Therefore, I would recommend you open the `/home/nvidia/route_1` directory and click on the first image (`frame_000000.png`) and then using your arrow keys navigate through all the images making sure there is only a small jump between consecutive images. Trust me you will know if there is a significant jump in your teach dataset.

### Step 3: Preprocess Teach Images
We now need to preprocees the teach images. To do this run the following:

    rosrun carlie_teach_and_repeat preprocess_teach_images ~/Documents/route_1/dataset.txt

This `preprocess_teach_images` script is taking in a path to a teach dataset file and will produce a new route directory with _processed appended (e.g. route_1 will become route_1_processed) which will contain the preprocessed images and a copy of the dataset.txt.

### Step 4: Repeat Phase

It is now time to run the repeat phase. Return Carlie to the starting position and run the following command:

    roslaunch carlie_teach_and_repeat repeat.launch

Now hold down the autonomous e-stop button (left bumper), but be ready to take your finger off at any moment. Also, stay close to Carlie that Gamepad only has a range of about 5-10m. Hopefully Carlie will autonomously navigate the route. The current example does have problem with tight corners, corners in general can be troublesome for teach and repeat.


## LIDAR Mapping

The LIDAR Mapping is a utility that can be used to create a occupancy grid map of your environment. This application utilises the [ROS SLAM Toolbox](http://wiki.ros.org/slam_toolbox). This capability can be run via the command line or through the web interface. To run the utility via the command line first open up RVIZ, if you wish to be able to view the map,

    rosrun rviz rviz

Then, in another terminal, launch the mapping utility,

    roslaunch carlie_lidar_mapping_app lidar_mapping_app.launch

Once you have finished building the map you can save it by executing in another terminal

    rosservice call /slam_toolbox/save_map "name: {data: 'maps/<map_name>'}"
    rosservice call /slam_toolbox/serialize_map "maps/<map_name>"

Maps will be saved to the path `~/.ros/maps/<map_name>`