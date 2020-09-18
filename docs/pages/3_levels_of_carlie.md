---
sort: 7
---

# The 3 Levels of Carlie
The Carlie platform exists of 3 levels in both its hardware and software design. These levels are:

1. The low-level layer
2. The base layer
3. The sensor layer

We have developed these layers to allow you to develop the fourth level, the application layer, as easily as possible. However, we have also developed Carlie with this 3 level principle to allow you to change components easily, such as the computation module or the sensors. Please note, we will sometimes use the term 'high-level' to refer to the combination of the base and sensor layers.

## The Low-Level Layer
The low-level layer consists of all the hardware and software components required to make the platform physically move. This includes:
* A power management system
* A Vedder Electronic Speed Control (VESC) board to manage the brushless motor
* A micro-controller to interface to various low-level hardware components, such as the VESC and GPS
* A tachometer, steering angle, IMU, GPS and proximity sensors
* A control interface via the standard RC remote (meaning a computer does not need to be connected to allow Carlie to be tele-operated)
* A motor emergency stop button

The micro-controller acts as a ROS node and publishes and subscribes to the following topics:

* The **/carlie/ackermann_cmd** topic - contains the control values to be performed, such as the linear velocity and steering angle.
* blah blah blah

The Carlie platform could have been built without the micro-controller, but this would mean the computer would have been required to connect with all the above sensors listed as well as the VESC. By having a micro-controller take care of these interfaces and to perform some preliminary filtering/analysis of the data, it means that only a single connection is required by the computer to control the movement of the platform. Additionally, this means that the computer can easily be swapped out for a different device and all that is required to control the car is a single USB and the carlie_base ROS package to be installed. The figure below shows how the low-level hardware components are connected. 

![](assets/HardwareArchitecture.png)

## The Base Layer
The base layer contains the computer as well as the ROS software required to communicate with the low-level hardware. Therefore, this base layer controls the movement of the platform via a computer interfacing with the low-level layer. The base layer consists of the computer and two ROS packages, [**carlie_msgs**](https://github.com/RoboticVisionOrg/carlie_msgs) and [**carlie_base**](https://github.com/RoboticVisionOrg/carlie_base).

The [**carlie_msgs**](https://github.com/RoboticVisionOrg/carlie_msgs) ROS package contains the message definitions for custom topics used to transfer data between the computer and the micro-controller on the low level. These message types are:

* The **CarlieConfig** and **CarlieConfigStamped** - contains data/values used to configure characteristics of the platform, such as the maximum velocity.
* The **CarlieRawMotionData** and **CarlieRawMotionDataStamped** - contains raw tachometer and motor RPM values provided by the VESC.
* The **CarlieRawOdom** and **CarlieRawOdomStamped** - contains the current estimate for the linear and angular velocity computed by the micro-controller (the standard ROS odometry message was to data heavy to be passed between the computer and the micro-controller at suitable frequencies).
* The **CarlieRawProximityData** and **CarlieRawProximityDataStamped** - contains the raw proximity data provided by the short range LIDAR sensors.
* The **CarlieStatus** and **CarlieStatusStamped** - contains data/values stating the current state of the low-level system, such as whether movement is currently enabled.

The [**carlie_base**](https://github.com/RoboticVisionOrg/carlie_base) ROS package contains three nodes and a singular launch file. The three nodes are:

* The **config_node** - attempts to set the Carlie config values on the micro-controller to be those specified by the ROS parameter server.
* The **low_level_converter_node** - converts topics coming from the micro-controller that are not in standard ROS message formats, such as the raw odometry data.
* The **driver_node** - which listens to the *joy* and */carlie/ackermann_cmd/autonomous* topics and publishes the */carlie/ackermann_cmd*, */carlie/ackermann_cmd/teleop* and */carlie/estop/gamepad* topics. The node also multiplexes the right- and left-bumpers of the Logitech F710 gamepad to enable hassle-free transition between the tele-operation and autonomous modes. If the right-bumper is held-down the */carlie/ackermann_cmd/teleop* topic is copied into the */carlie/ackermann_cmd* topic which is then passed onto the micro-controller. If the left-bumper is held-down the */carlie/ackermann_cmd/autonomous* topic is copied into the */carlie/ackermann_cmd* topic and then passed onto the micro-controller. If both right- and left-bumper or neither button is held down the */carlie/estop/gamepad* topic is set to false and the speed within the */carlie/ackermann_cmd* topic is set to zero.

The figure below shows the nodes, topics and connections when the base layer and low-level ROS packages are running.


## The Sensor Layer