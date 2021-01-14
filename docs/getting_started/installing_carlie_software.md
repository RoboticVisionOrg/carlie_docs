---
sort: 2
---

# Installing Carlie Software
**Note**: *If you built Carlie using the provided OS Image you can skip this step*.

To install software please execute the following. First import the GPG Key using the following command

    sudo -E apt-key adv --keyserver hkp://keyserver.ubuntu.com --recv-key 5B76C9B0

Add the QCR repository to the apt sources list directory:

    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://packages.qcr.ai $(lsb_release -sc) main" > /etc/apt/sources.list.d/acrv-latest.list'

Update your packages list

    sudo apt list

Then finally install the Carlie software packages

    sudo apt install ros-melodic-carlie

**Note**: Only ROS Melodic is currently supported.

