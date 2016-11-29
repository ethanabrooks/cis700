# Arm Simulation in Gazebo

## Setting up a 4-axis arm simulator

*This assumes an existing Catkin Workspace (see [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).*

First, install dependencies:
```
sudo apt-get install ros-indigo-moveit-full
sudo apt-get install ros-indigo-ur-kinematics
sudo apt-get install ros-indigo-ur5-moveit-config
sudo apt-get install ros-indigo-gazebo-ros
rosdep update
```

To install the required repositories from Github, `cd` into the root directory of your workspace and

```
source devel/setup.bash
cd src
git clone https://github.com/ros-industrial/universal_robot.git
cd ..
source devel/setup.bash
catkin_make
cd src
git clone -b indigo-devel https://github.com/ros-planning/moveit_core
cd ..
source devel/setup.bash
catkin_make
```

(Go grab a coffee, or maybe a light snack.) DO NOT RUN ROSCORE. Now, to open Gazebo, type:

`roslaunch ur_gazebo ur5.launch limited:=true`

This should open a GUI that displays a UR5 robotic arm.

### Teleop control

This will run a small python script that takes user input joint angles and executes them in gazebo:

`rosrun ur_driver test_move.py`

### Planning and executing random arm positions:

With gazebo open in one terminal, we need to initialize MoveIt planning. To do so, open a second terminal and type:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true`

Next, we need to open RViz. In a third terminal, type:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

This will open a GUI also displaying the UR5 arm. On the lower left, in the "Motion Planning" section, click on the "Planning" tab. Click on "Select Goal State", in the drop-down, select "<random valid>" and click "Update." This should move a yellow version of the arm to a new position. To prevent Gazbo from crashing, ensure that this new position does not collide with the floor. Now click "Plan and Execute." If all goes well, Gazebo should show the arm moving into the new position.

If RViz displays "failed" under "Plan and Execute," you need to reset the arm. To do so, click "Select Start State" (above "Select Goal State"), select "<same as goal>" from the drop-down, and click "update." Then click "Plan and Execute." If this does not work, just quit and restart RViz.

Finally, if you did try to send the arm to a position that collides with the floor, the arm will try to execute in Gazebo but then flop around, causing Gazebo to become unresponsive. In this case, you need to restart Gazebo.

# The process for our arm electronics design

Initial investigation went to building an Arduino module to control the arm as per this demo: http://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port
This approach had the advantage of removing the controller for the arm from the NUC to a dedicated system that would be able to run independently for basic trajectories and free resources from the NUC. Additionally, the Arduino interface would provide a convenient access point for integrating any new electro-mechanical sensors via its GPIO pins that the NUC currently lacks. The biggest problem was that the Arduino platform does not natively support the half-duplex serial encoding of data between the Dynamixel motors and the host, nor does it support rs485 serial encoding for the RX and EX motors that are needed for their higher strength to weight ratio.

Initial work on developing a dedicated messaging packet structure for setting target positions over ROS was successful when using the Arbotis controller board. However, these electronics could not be used as they do not generate the required wattage for the AX-12 models at peak torque, and do not support the RX and EX models at all.

After several hardware delays, initial vetting of the library posted in the above blog post did not result in the desired behavior, possibly because the interface library it depends on is significantly out of date, poorly documented, and is itself a hack to standard serial interface as it requires the use of an additional IO pin for negotiating transmission and receive packets through the tri-state buffer. Other circuits for converting a full-duplex to half-duplex signal without the extra control pin from the Arduino were equally unsuccessful. Even if this approach did work, additional motor libraries would need to be developed to modify the AX-12A API to handle the RX and EX model motors as both models have extended capabilities.

Currently a new set of electronics is being ordered that utilizes Trossin Robotics own library, which supports many languages as well as ROS. The advantages is that hopefully this will drastically reduce development overhead and complexity, at the cost of relying on the NUC for monitoring and control of kinematic model, and not providing any additional IO extensibility (though the Arduino is still available should the need to include more sensors arise). The new electronics will require two USB to Serial adapters (1 for AX motors, 1 for both the RX and EX), as well as two power distribution boards for each pair of motors. Fortunately these electronics are only slightly more expensive that the Arduino configuration, but do not rely on hacking the electronics for serial communication.

Preliminary investigation into the library shows that it is very easy to install and configure, though tests were run on a full duplex usb to serial adapter, and again electronics issues were encountered when trying to convert to half duplex.
