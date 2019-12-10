# Starting Up and Driving NightRider

## Turning On NightRider
To turn on NightRider press the *on button* located on the shell. Pressing the on button provides power to all the various systems and automatically turns on the computer. After a short delay (1-2 minutes) the screen will turn on. The default username and password to the computer are both *nvidia*.

As part of the start-up sequence the computer will automatically launch the following systems:

* NightRider Base System - this is required to allow the computer to communicate with the low level hardware (motion control, IMU, GPS, short-range LIDAR sensors) as well as provides the tele-operation via the gamepad and the hassle-free switch between tele-operation and autonomous modes.

## Turning Off Nightrider
To turn off NightRider we recommend you first shut down the computer in the usual fashion. Once the computer has shut down you can press the off button located on the shell. Pressing the off button disconnects power from all the systems.

**Note:** if you shut down the computer but forget to press the power button the power system will automatically power off if the battery goes too low. See the [Power Management](power_management.md) for more information.

## Driving NightRider
NightRider can be controlled in three different ways out-of-the-box. These three ways are:

* Tele-operated via the Logitech F710 gamepad (default),
* Tele-operated via the Traxxas RC Remote that came with the Traxxas Platform (which NightRider is built upon), or
* Autonomously via computer generated control commands.

### **Tele-Operation via Logitech F710 Gamepad**
A minute or two after the computer has started up you should be able to drive NightRider using the Logitech Gamepad. The controls are as follows:

* Right Bumper - is a deadman switch. It must be pressed at all times when tele-operating the robot. Releasing it will stop the vehicle.
* Right Joystick - is used to control the steering.
* Left Joystick - is used to control the linear velocity.

You may notice a slight left or right drift when trying to drive straight, please see the [Config and Calibration](#config-and-calibration) routine below on how to remedy this.

**Safety Warning!** We have noticed that the gamepad has a reliable range of about 5 metres. After 5 metres there is some extremely noticeable lag and dropout, and releasing of the deadman switch may not result in an immediate stop. If you wish to be further than 5 metres away from the vehicle while tele-operating we recommend using the RC remote instead.

### **Tele-Operation via Standard RC Remote**
NightRider can also be operated via the standard RC Remote. While this method does allow for longer range operation, it does mean you cannot have additional capabilities tied to that remote (i.e. a deadman switch or a button to change states etc.). In order to control the platform using the RC Remote you will need to flick the switch that resides next to the Teensy on the Low Level Control Board (white PCB). To do this:

1. Lift up the lid of the platform.
2. Locate the switch next to the Teensy. You will need to look in from the side.
3. Flick the switch. 
4. You should now be able to operate the vehicle using the RC remote.

While the platform is in this mode, which is dictated by the switch, the car can not receive autonomous control commands. This was a design choice due to safety, as we wanted the user to have to hold down a deadman switch for safety.

### **Autonomous Driving**
At some point you will want to the car to perform some autonomous actions. We have made the transistion from Logitech tele-operation to autonomous mode hassle free. To transition from Logitech tele-operation mode to autonomous mode all you need to do is:

1. Bring the car to a halt. 
2. Stop holding the Right Bumper, which we will now call the tele-operation deadman switch.
3. Press and hold the Left Bumper, which we will now call the autonomous deadman switch.
4. If there are autonomous car commands being generated then the car would now listen to them rather than the tele-operatio commands.

This should allow you to easily switch between tele-operation and autonomous modes and allow you to easily reset experiments. The HSV Blob Following Demo shows how you can generate autonomous control commands.

**Note:** for ROS users what we have done is created an exclusive-or multiplexer using the left and right bumpers of the Logitech Gamepad which forwards either the */nightrider/ackermann_cmd/teleop* or */nightrider/ackermann_cmd/autonomous* down to the low level hardware via the */nightrider/ackemann_cmd* topic, depending on which bumper is currently held. All three messages are of type *AckermannDrive*. 

## Config and Calibration
The platform comes with some default parameters which set aspects such as acceleration and velocity limits. Fortunately these can all be easily changed through a YAML config file. The config file is located at **insert config file location**. In this file the config parameters/values will be commented out. You will need to uncomment them to override the defaults (the defaults are located at **file location**). To change the file:

1. Open up a terminal, either via the applications GUI or by pressing `ctrl+alt+t` on the keyboard.
2. Change directory to the location of the config file `cd file_location`.
3. Use Nano, or Vim, to edit the file `nano file_name`.
4. Uncomment the parameter you wish to specify and change the value.
5. Once completed save the file. In Nano this is done by pressing `ctrl+x` and then entering `y` to the question do you wish to overwrite this file.

The config values and their meaning are below. The following values you will wish to change depending on your application/environment:

* **max_reverse_velocity** - this is the maximum reverse velocity you want the car to be able to go; specified in m/s and does require a negative sign (e.g. -1m/s).
* **max_forward_velocity** - this is the maximum forward velocity you want the platform to go; specified in m/s.
* **acceleration** - is the acceleration limit; specified in m/s/s.
* **ignore_proximity_sensors** - set this to true to ignore the four short range LIDAR bumper sensors. The sensors will still be measured and reported but the vehicle will not stop. Current default is true as need to write a filter for spurious false-positives when the car is undulating.
* **collisision_detection_distance** - the approximate distance in millimetres for when the LIDAR bumper sensors should stop the car.

The parameters associated with the low level odometry covariance you may wish to change, but you should not need to.
* **covar_position_<value>** - the covariance for various element of the reported odometry from the low level.

The following parameters are specific to your exact hardware and while the defaults will probably be sufficient you may wish to calibrate them (see below). The parameter you will probably wish to calibrate is the center_steering_angle, else you may get a left/right drift when wanting to drive straight.
* **min_throttle_pulse_width** - the minimum throttle (linear velocity) pulse width generated by the RC remote; specified in micro-seconds should be approximately 1000us.
* **max_throttle_pulse_width** - the maximum throttle (linear velocity) pulse width generated by the RC remote; specified in micro-seconds should be approximately 2000us.
* **center_throttle_pulse_width** - the pulse generated when the RC throttle is in its neutral position; specified in micro-seconds should be approximately 1500us.
* **min_steering_angle** - the minimum steering angle possible by the car (wheels are facing to the right when looking in direction of travel); specified in radians and should be negative (e.g. -0.7854 radians).
* **max_steering_angle** - the maximum steering angle possible by the car (wheels are facing to the left when looking in direction of travel); specified in radians default.
* **center_steering_angle** - placing the steering servo so 100 degrees corresponds to exactly straight is impossible, hence this parameter. This specifies the center position of the steering; units are in radians and should be approximately 1.75 radians (100 degrees).
* **min_steering_angle_pulse_width** - the minimum pulse width generated by the RC remote when steering; specified in micro-seconds should be approximately 1000us.
* **max_steering_angle_pulse_width** - the maximum pulse width generated by the RC remote when steering; specified in micro-seconds should be approximately 2000us.
* **lidar_front_right_address** - the I2C address of the front right LIDAR bumper sensor.
* **lidar_front_left_address** - the I2C address of the front left LIDAR bumper sensor.
* **lidar_back_right_address** - the I2C address of the back right LIDAR bumper sensor.
* **lidar_back_left_address** - the I2C address of the back left LIDAR bumper sensor.

The following parameters should not need to be changed. They are merely broken out as config values for ease of use, so the Teensy does not need to be reprogrammed each time.
* **wheel_base** - the distance between the front and rear axle of the platform; specified in metres.
* **wheel_radius** - the radius of the wheels; specified in metres defaults to 0.059m.
* **gear_ratio_wheel_2_motor** - the gear ratio from the wheel to the motor, should be a value greater than 1.
* **motor_poles** - the number of motor poles, as specified by the VESC documentation. We believe the VESC documentation is wrong as the motor should have more than 1 pole, but the conversion required to go from E-RPM to RPM is only sensible if you use a value of 1 here.
* **motor_ticks_per_revolution - the number of motor ticks per single revolution.
* **battery_voltage_offset** - the offset between the battery voltage measured by the VESC and the actual voltage due to voltage offsets caused by components such as diodes (this should be approximately constant); specified in volts.

The easiest way to calibrate the steering angle is as follows:

1. Turn on the car and make sure it is in Logitech Tele-Operation mode.
2. Manually drive the car forward a few metres and see if there is any left/right drift.
3. If there is drift slightly alter the center_steering_angle parameter within the local config file (<file_location>). If it is steering left add(?) to this value, if right subtract(?) from it.
4. You will need to restart the nightrider base package to update the parameter. This can be done by `pm2 restart nightrider_base`.
5. Give the car 0.5-1 minute to restart the base and then repeat the process. You will most likely not be able to get a car that can drive perfectly straight over a long straight due to the fact the discritization of the angles the servo can physically achieve.

The default pulse width RC remote parameters will do in most cases. However, we do recommend you calibrate them. To calibrate:

1. Somehow upload the calibration firmware. Gavin this is on you
2. James how do they use it?

**Tip:** on the side of the RC remote there is a steering trim knob. If this is steering trim does not align with the value specified in the *center_throttle_pulse_width* parameter your car will drift left or right. Changing the steering trim will change the wheel alignment. However, we recommend first set the *center_steering_angle* parameter as outlined below. After setting this parameter set the steering trim knob to the neutral position, or as close as possible (1500us pulse width). Then calibrate the pulse width parameters, again see below for calibration routine. Then when you are using the RC remote changing the steering trim should not cause errors in reported measurements due to a mismatch in offsets.