# ros_wiring_template
This repository contains a template ROS2 node which implements the Arduino API for I2C (and soon GPIO, SPI and Serial) to enable using Ardunio C++ libraries in order to implement a ROS2 node for specific sensors or actuators.

## Using this ROS2 Node Template
1. On Github, select the `Use this template` button. This will begin creating a fork in your github account.
1. Select a name for your fork, representing the node you are creating. In the examples below, I'll use `ros_firefly`, but this is your ROS node's name.
1. Once it has been forked into your github account, navigate to your ROS2 workspace.
1. clone your fork into your workspace **recursively** - `git clone --recursive https://github.com/<your github>/ros_firefly`
1. Update the following entries in your fork:
    * `package.xml` - change `<name>ros_wiring_template</name>` to `<name>ros_firefly</name>`
    * `CMakeLists.txt` - change `project(ros_wiring_template)` to `project(ros_firefly)`
    * Rename `ros_wiring_template.launch.py` to `ros_firefly.launch.py`
    * In `ros_firefly.launch.py`, change `'ros_wiring_template'` to `'ros_firefly'`

    Optionally
    * in `main.cpp` change `class I2CPublisher` to `class ROSFireFly` 
    * in `main.cpp` change `Node("i2cpublisher")` to `Node("FireFly")`
    * in `main.cpp` change `std::make_shared<I2CPublisher>()` to `std::make_shared<ROSFireFly>()`


1. push your changes to your repository `git add *` then `git push`
    
