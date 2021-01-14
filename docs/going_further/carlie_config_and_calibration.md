---
sort: 2
---

# Carlie Config and Calibration

## Config Parameters
The platform comes with some default parameters which set aspects such as acceleration and velocity safety limits. Fortunately these can all be easily changed through a YAML config file. The config file is located at */etc/carlie.conf*. In this file the config parameters/values will be commented out. You will need to uncomment them to override the defaults (the defaults are located at *ros_path/carlie_base/config/carlie_config.yaml*). To change the file:

1. Open up a terminal, either via the applications GUI or by pressing `ctrl+alt+t` on the keyboard.
3. Use Nano, or Vim, to edit the file `nano /etc/carlie.conf`.
4. Uncomment the parameter you wish to specify and change the value.
5. Once completed save the file. In Nano this is done by pressing `ctrl+x` and then entering `y` to the question do you wish to overwrite this file.

The config values and their meaning are below. The config parameters are split into six sections. In the ROS parameter server each of these parameters will be within the carlie namespace. This means to access them you will need to prepend the names with */carlie/* (e.g. /carlie/velocity_safety_limits).

### Application Parameters
The following parameters you may wish to change depending on your application and environment. These values can be updated at runtime.

* **velocity_safety_limits** - a two element vector containing the maximum allowable reverse and forward velocity in m/s. The reverse velocity requires a negative sign (e.g. -1m/s). Default is [-1, 2].
* **acceleration_safety_limit** - a single float value dictating the maximum allowable acceleration in m/s^2. Default is 1.5.
* **steering_angle_safety_limit** - a single float which sets the maximum allowable steering angle in radians. Default is 0.78.
* **ignore_proximity_sensors** - a boolean value used to ignore the four short range LIDAR bumper sensors. Default is true. If the sensors are ignored, they will still be measured and reported but the vehicle will not stop. *Current default is true as need to write a filter for spurious false-positives when the car is undulating*.
* **collisision_detection_distance** - the approximate distance in millimetres for when the LIDAR bumper sensors should stop the car.

### Control Parameters
The following parameters affect the control values of Carlie and are hardware dependent. You may wish to calibrate these values after unboxing Carlie, but the defaults typically suffice. If you do wish to calibrate them see [below](#calibrating-carlie-control-parameters) and they should only need to be calibrated once. These values can be updated at runtime.

* **rcremote_throttle_pulse_widths** - a three element vector representing the pulse widths, in milliseconds, for the throttle channel from the RC remote [lower, neutral, upper]. Default is [1000, 1500, 2000].
* **rcremote_steering_angle_pulse_widths** - a three element vector representing the pulse widths, in milliseconds, for the steering channel from the RC remote [lower, neutral, upper]. Default is [1000, 1500, 2000].
* **physical_steering_limits** - a two element vector containing the physical steering limits, in radians, of the wheels [lower_limit, upper_limit]. The lower_limit should be negative. Default is [-0.7854, 0.7854].
* **center_steering_angle** - A float representing the angle, in radians, that the SC servo reports when the wheels are pointing straight. Default is 1.745.

### Hardware Config
The following parameters are hardware dependent, but should not need to be changed. These values can be updated at runtime.

* **wheel_base** - a float specifying the distance, in metres, between the front and rear axle of the platform. Default is 0.312
* **wheel_radius** - a float dictating the radius of the wheels, specified in metres. Default is 0.059.
* **gear_ratio_wheel_to_motor** - a float representing the gear ratio from the wheel to the motor, should be a value greater than 1. Default is 40.014.
* **motor_poles** - an integer representing the number of motor poles, as specified by the VESC documentation. Default is 1. *We believe the VESC documentation is wrong as the motor should have more than 1 pole, but the conversion required to go from E-RPM to RPM is only sensible if you use a value of 1 here.*
* **motor_ticks_per_revolution** - the number of motor ticks per single revolution.
* **battery_voltage_offset** - the offset between the battery voltage measured by the VESC and the actual voltage due to voltage offsets caused by components such as diodes (this should be approximately constant); specified in volts.
* **short_range_lidar_addresses** - a four element vector containing the I2C addresses for the four short range LiDAR proximity sensors [front_right, front_left, back_right, back_left]. Default is [10, 11, 9, 8]. The Teensy will need to be reset, power cycled, for a parameter value change to take affect.

### Odometry EKF Filter Config
The following parameters are used within the odometry EKF filter, which filters the encoder and IMU measurements. These values **cannot** be updated at runtime and will require a restart of carlie_base service.

* **odom_filter_publish_tf** - a boolean specifying if the odometry filter should publish a odom-to-base_link transform using the filtered odometry. Default is true.
* **odom_frame** - a string for the frame_id within the odometry topic and tf, if published. Default is odom.
* **base_frame** - a string for the child_frame_id within the odometry topic and tf, if published. Default is base_line.
* **filtered_odom_msg_freq** - an integer specifying the rate at which to publish the filtered odometry message and transform. Default is 30Hz.
* **filter_odom_timeout** - a float dictating how long to wait before pausing the EKF prediction step if an odometry message has not been received. Default is 1 second.
* **process_noise_covar** - a 8x8 matrix representing the process covariance. States are ordered as [x, y, yaw, vx, vy, vyaw, ax, ay]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults.
* **initial_covar** - a 8x8 matrix representing the initial covariance for each state. States are ordered as [x, y, yaw, vx, vy, vyaw, ax, ay]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 

### Odometry Covariances
The following parameters are used to set the odometry covariances. These values **cannot** be updated at runtime and will require a restart of carlie_base service.

* **odom_pose_covar** - a 6x6 matrix representing the odometry pose covariances. States are ordered as [x, y, z, roll, pitch, yaw]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 
* **odom_vel_covar** - a 6x6 matrix representing the odometry velocity covariances. States are ordered as [vx, vy, vz, vroll, vpitch, vyaw]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 


### IMU Sensor Config
The following parameters are used to set the IMU covariances. These values **cannot** be updated at runtime and will require a restart of carlie_base service.

* **imu_covar_angle** - a 3x3 matrix representing the IMU angle covariances. States are ordered as [roll, pitch, yaw]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 
* **imu_covar_angVel** - a 3x3 matrix representing the IMU angular velocity covariances. States are ordered as [vroll, vpitch, vyaw]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 
* **imu_covar_linAcc** - a 3x3 matrix representing the IMU linear acceleration covariances. States are ordered as [ax, ay, az]. See *ros_path/carlie_base/config/carlie_config.yaml* for defaults. 


## Calibrating Carlie Control Parameters
To calibrate the control parameters perform the following. You will require a serial monitor program. We recommend you install the [Arduino IDE](https://www.arduino.cc/en/software) along with the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) add-on and use the Arduino serial monitor, if it is not already available.

1. Turn on the car and make sure the E-Stop button, the one on the shell, is engaged (pressed down).
2. Change the Teensy firmware over into calibration mode by opening up a terminal and running the following command `sudo CarlieInstallTeensyFirmware /var/lib/carlie/calibration.hex`
3. Open up the Arduino IDE and select *Tools > Port > /dev/ttyACM0*
4. Open up the Arduino Serial Monitor (icon in top right corner) and set the baud to 57600. You should now see data been printed to the screen.
5. Calibrate the parameter, see below for details pertaining to each parameter, and set the value within the config file */etc/carlie.conf*.
6. Move onto the next parameter by sending the command `#on`.
7. Once you have finished calibrating all parameters make sure you upload the Carlie Teensy firmware. To do this run the following `sudo CarlieInstallTeensyFirmware`

To calibrate the center steering angle (center_steering_angle):

1. Ensure the serial monitor is showing the current current steering angle in radians and degrees.
2. To change the wheel angle send the command `w<val>`, where `<val>` is an integer degree value (e.g. w100 for 100 degrees). Wait a couple of seconds for the wheel angle to change.
3. Keep performing step 2 until you feel the wheels are pointing straight. An additional calibration procedure to refine the angle is discussed below.
4. Once you are happy set the value within the config file.

To calibrate the RC throttle and steering angle parameters (rcremote_throttle_pulse_widths, rcremote_steering_angle_pulse_widths):

1. Ensure the current data is showing the appropriate parameter
2. Turn on the RC remote, if it is not already on. The platform does not need to be in RC mode for this calibration.
3. Move the throttle or steering input to their mimimum and maximum position. 
4. Record the mimimum and maximum values and set these within the config file. *Should be approximately 1000 and 2000*
5. Leave the throttle or steering input in its neutral position, record this value and set it within the config file. *Should be approximately 1500*

**Note:** On the side of the RC remote, near the steering angle input, there is a *Steering Trim* knob. This will affect the value of the neutral steering position. We recommend you set this to be in the 12 o'clock position, or as close possible, and then do not move it. 

To further refine the center steering angle:

1. Turn on the car and make sure it is in Logitech Tele-Operation mode.
2. Manually drive the car forward a few metres and see if there is any left/right drift.
3. If there is drift, slightly alter the center_steering_angle parameter by opening up a terminal and running the command `rosparam set /carlie/center_steering_angle <value>`. If it is steering left subtract from this value, if the car is drifting right add to this value. You can get the current value by running the command `rosparam get /carlie/center_steering_angle`. You will need to hold the deadman switch to see the change in the wheels after altering the parameter.
4. Repeat steps 2 and 3 until you are happy with the result. You will most likely not be able to get a car that can drive perfectly straight over a long straight due to the fact the angles of the servo are discretised and so there is a minimum resolution that can physically be achieved.
5. Once you are happy with the result you will need to make the change official by updating the config file.