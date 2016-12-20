# Arm Simulation in Gazebo

## Setting up a 4-axis arm simulator

*This assumes an existing Catkin Workspace (see [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).*

First, install dependencies:
```
sudo apt-get install ros-indigo-moveit-full
sudo apt-get install ros-indigo-ur-kinematics
sudo apt-get install ros-indigo-ur5-moveit-config
sudo apt-get install ros-indigo-gazebo-ros
sudo apt-get install ros-indigo-gazebo-ros-control
sudo apt-get install ros-indigo-effort-controllers
sudo apt-get install ros-indigo-joint-state-controller
sudo apt-get install ros-indigo-joint-trajectory-controller
sudo apt-get install ros-indigo-gazebo-ros-pkgs
sudo apt-get install ros-indigo-ros-controllers
sudo apt-get install ros-indigo-moveit-resources
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

To have dynamixel servos follow the joint state topic:
`roslaunch move_arm follow_arm.launch`

### Planning and executing random arm positions:
With gazebo open in one terminal, we need to initialize MoveIt planning. To do so, open a second terminal and type:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true`

Next, we need to open RViz. In a third terminal, type:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

This will open a GUI also displaying the UR5 arm. On the lower left, in the "Motion Planning" section, click on the "Planning" tab. Click on "Select Goal State", in the drop-down, select "<random valid>" and click "Update." This should move a yellow version of the arm to a new position. To prevent Gazbo from crashing, ensure that this new position does not collide with the floor. Now click "Plan and Execute." If all goes well, Gazebo should show the arm moving into the new position.

If RViz displays "failed" under "Plan and Execute," you need to reset the arm. To do so, click "Select Start State" (above "Select Goal State"), select "<same as goal>" from the drop-down, and click "update." Then click "Plan and Execute." If this does not work, just quit and restart RViz.

Finally, if you did try to send the arm to a position that collides with the floor, the arm will try to execute in Gazebo but then flop around, causing Gazebo to become unresponsive. In this case, you need to restart Gazebo.

# Motor config
If ros cannot open the serial controller, make sure that the current user has ownership:
`sudo chown host /dev/ttyUSB0`

Conversion from encoders to degrees:  
Motor 1:  
  1500 = 90 degrees (fully extended)  
  3000 = 180 degrees (straight up)  
Motor 2:  
  205 = 90 degrees (rotate left)  
  820 = 270 degrees (rotate right)  

To subscribe to the ur5 motion planner and send motor commands:
`roslaunch move_arm follow_arm.launch`
=======
# Grasping
## Setup camera
### Option 1: RealSense
More detailed instructions can be found [here](http://wiki.ros.org/RealSense).
First, you need to install `librealsense`. This process is described at this
[website](http://wiki.ros.org/librealsense), but the instructions are reproduced
here:
```
wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
bash ./enable_kernel_sources.s
sudo apt-get --reinstall install 'ros-indigo-librealsense'
```
Next, install `realsense_camera`. Again, more detailed instructions can be found
at [this site](http://wiki.ros.org/realsense_camera). Simply run
```
sudo apt-get install ros-indigo-realsense-camera
```

... this is where we gave up.


### Option 2: Kinect
```
sudo apt-get install freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev doxygen graphviz mono-complete
```
Create a `kinect` folder in your home directory to store installation source of drivers:
```
mkdir -p ~/kinect
cd ~/kinect
```
Download from the OpenNI modules. Make sure you get the `unstable` branch:
```
git clone https://github.com/OpenNI/OpenNI.git
cd OpenNI
git checkout unstable
cd Platform/Linux/CreateRedist
./RedistMaker
```
If that succeeds, there a `Redist` folder should be created with an install script. Run it:
```
cd ~kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.8.5/
sudo ./install.sh
```
Many blogs recommend installing from https://github.com/avin2/SensorKinect.git
This didn't work for me, but the following the following repo from ph4m worked:
```
cd ~/kinect
git clone https://github.com/ph4m/SensorKinect.git
git checkout unstable
```
Install it
```
cd SensorKinect/Platform/Linux/CreateRedist
./RedistMaker
```
If that succeeds, there a `Redist` folder should be created with an install script. Run it:
```
cd ~/kinect/SensorKinect/Platform/Linux/Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
```

## Agile_grasp
`cd` to your catkin workspace.
```
cd src/
git clone https://github.com/atenpas/agile_grasp
cd ..
catkin_make
```
Make sure this previous command succeeded. The percentage should go to 100%.
Next, start publishing data from the Kinect:
```
roslaunch openni_launch openni.launch
```
Now we are ready to run `agile_grasp`:
```
roslaunch agile_grasp single_camera_grasps.launch
```
To visualize the grasp points, open rviz:
```
rosrun rviz rviz
```
Type ctrl+O and select `agile_grasp/rviz/single_camera.rviz`.
You should see a camera image from the Kinect and little bristly looking things representing grasp points should start popping up all over the place.

## Moving arm to end-effector position
In order to enable writing to the USB port, run
```
sudo chown host /dev/ttyUSB0
```
This may not be the name for the USB port that your using, so you'll want to double check that.
Next, start `roscore` and load the ur5 robot description by running
```
roslaunch ur_gazebo ur5.launch limited:=true
```
Next, in a separate window, turn on the servos and begin listening to joint states by running
```
roslaunch move_arm follow_arm.launch
```
To start MoveIt, which performs planning and inverse kinematics, in a separate window, run
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```
Finally, to run the `plan_trajectory` script, in a seaparate window, run
```
 rosrun move_arm plan_trajectory.py
 ```

### Troubleshooting
A good first step is to kill processes in all windows and restart them in the order listed in this document.

If that doesn't work, especially if some of the motors are unresponsive, unplug and replug all motors.

If you keep getting
```
[TxRxResult] There is no status packet!
```
Reenter
``` 
sudo chown host /dev/ttyUSB0
```

If the arm is behaving strangely, check the model as depicted in Gazebo. You may need to restart it by rerunning

```
roslaunch ur_gazebo ur5.launch limited:=true
```

Ensure that the 4-pin connector has 19V and the 3-pin connector has 12V with at least 2A.

Note that the end-effector position is hard-coded in the file. Feel free to change it although unreachable positions may cause problems.

# The process for our arm electronics design

Initial investigation went to building an Arduino module to control the arm as per this demo: http://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port
This approach had the advantage of removing the controller for the arm from the NUC to a dedicated system that would be able to run independently for basic trajectories and free resources from the NUC. Additionally, the Arduino interface would provide a convenient access point for integrating any new electro-mechanical sensors via its GPIO pins that the NUC currently lacks. The biggest problem was that the Arduino platform does not natively support the half-duplex serial encoding of data between the Dynamixel motors and the host, nor does it support rs485 serial encoding for the RX and EX motors that are needed for their higher strength to weight ratio.

Initial work on developing a dedicated messaging packet structure for setting target positions over ROS was successful when using the Arbotis controller board. However, these electronics could not be used as they do not generate the required wattage for the AX-12 models at peak torque, and do not support the RX and EX models at all.

After several hardware delays, initial vetting of the library posted in the above blog post did not result in the desired behavior, possibly because the interface library it depends on is significantly out of date, poorly documented, and is itself a hack to standard serial interface as it requires the use of an additional IO pin for negotiating transmission and receive packets through the tri-state buffer. Other circuits for converting a full-duplex to half-duplex signal without the extra control pin from the Arduino were equally unsuccessful. Even if this approach did work, additional motor libraries would need to be developed to modify the AX-12A API to handle the RX and EX model motors as both models have extended capabilities.

Currently a new set of electronics is being ordered that utilizes Trossin Robotics own library, which supports many languages as well as ROS. The advantages is that hopefully this will drastically reduce development overhead and complexity, at the cost of relying on the NUC for monitoring and control of kinematic model, and not providing any additional IO extensibility (though the Arduino is still available should the need to include more sensors arise). The new electronics will require two USB to Serial adapters (1 for AX motors, 1 for both the RX and EX), as well as two power distribution boards for each pair of motors. Fortunately these electronics are only slightly more expensive that the Arduino configuration, but do not rely on hacking the electronics for serial communication.

Preliminary investigation into the library shows that it is very easy to install and configure, though tests were run on a full duplex usb to serial adapter, and again electronics issues were encountered when trying to convert to half duplex.
