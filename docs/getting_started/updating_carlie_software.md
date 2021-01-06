---
sort: 2
---

# Updating Carlie Software

## Updating ROS Packages

Before going any futher we should ensure your Carlie platform has the most up-to-date software. To upgrade to the latest Carlie software, turn on Carlie and open up a terminal, by pressing `ctrl+alt+t`. Then in the terminal run the following commands:

    sudo apt update
    sudo apt upgrade
    sudo apt install ros-melodic-carlie

## Updating Carlie Ubuntu Image

Instructions to come. The Carlie team need to determine the easiest path for users.


## Reporting Issues

If you have problems with installing or running any of the Carlie softare please report an issue using Github's Issues within the appropriate package. For example, if the problem is in *carlie_base* please report at [Carlie Base Package Issues](https://github.com/RoboticVisionOrg/carlie_base/issues). If you are unsure which package is causing the issue please report the issue at [Carlie Package Issues](https://github.com/RoboticVisionOrg/carlie/issues).

If you do find and issue and have a software patch, or if you have developed a feature(s) that you wish to include in the carlie packages, please check out the [Developer Section](../going_further/developing_for_carlie) on how to do this.