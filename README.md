Installing and testing the katana 300
=============
1. Create a catkin workspace

mkdir -p $HOME/ROS_katana/src
cd $HOME/ROS_katana/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
cd src

2. Clone the repository

git clone https://github.com/JakaCikac/katana_300_ros

3. Install some dependencies

sudo apt-get install ros-hydro-control-msgs  ros-hydro-pr2-controllers libarmadillo-dev ros-hydro-convex-decomposition ros-hydro-moveit-msgs
Original readme: katana_driver

4. Make the packages

cd katana_300_ros
rosmake katana
rosmake katana_tutorials

5. Setup connection

If you are using a USB to serial converter:

sudo chmod a+rw /dev/ttyUSB0
sudo ln -sf /dev/ttyUSB0 /dev/ttyS3

Otherwise the port number ( dev/ttys# ) can be set in the file:
/katana/launch/katana_300_6m180.launch

6. Set the environment variables
Put the following in your .bashrc
cd
gedit .bashrc
At the end of the file copy: 

export KATANA_TYPE="katana_300_6m180"
source /opt/ros/hydro/setup.bash
source /home/ROS_katana/devel/setup.bash

Save and close gedit. 
source .bashrc

7. Launch the katana and start calibration
Open new terminal window and

roslaunch katana katana.launch

8. Start the trajectory server and client
Open new terminal window and

roslaunch katana_tutorials follow_joint_trajectory_client.launch

9. Send the start trajectory command 
Before running the following command make sure you are near the emergency stop button (or shutdown) and push it in case the Katana is headed for a collision.

Open new terminal window and

rostopic pub /katana/start_trajectory std_msgs/Bool 1

This should start the first trajectory with the Katana.

10. Return the Katana to the initial position
Open new terminal window and

rostopic pub /katana/start_trajectory2 std_msgs/Bool 1

The Katana should now return to the initial (calibration) position.

=============

This stack contains ROS hardware drivers, Gazebo plugins and other basic functionalities for the Neuronics Katana family of robot arms. Specifically, it provides: 

* JointTrajectory and FollowJointTrajectory execution on the physical arm (packages `katana`, `kni`, `katana_trajectory_filter`, `katana_msgs`),
* simulation of the Katana arm in Gazebo (packages `katana_gazebo_plugins`, `katana_arm_gazebo`),
* URDF descriptions (package `katana_description`),
* simple teleoperation (packages `katana_teleop`, `katana_joint_movement_adapter`), and
* some demo programs (package `katana_tutorials`).

For more information, visit the [katana_driver ROS wiki page](http://www.ros.org/wiki/katana_driver).
