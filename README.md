Installing and testing the katana 300
=============

* Create a catkin workspace

<pre><code>
mkdir -p $HOME/ROS_katana/src
cd $HOME/ROS_katana/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
cd src
</code></pre>

* Clone the repository
<pre><code>
git clone https://github.com/JakaCikac/katana_300_ros
</code></pre>

* Install some dependencies
<pre><code>
sudo apt-get install ros-hydro-control-msgs  ros-hydro-pr2-controllers libarmadillo-dev ros-hydro-convex-decomposition ros-hydro-moveit-msgs
</code></pre>

* Make the packages
<pre><code>
cd katana_300_ros
rosmake katana
rosmake katana_tutorials
</code></pre>

* Setup connection

If you are using a USB to serial converter:
<pre><code>
sudo chmod a+rw /dev/ttyUSB0
sudo ln -sf /dev/ttyUSB0 /dev/ttyS3
</code></pre>
Otherwise the port number ( dev/ttys# ) can be set in the file:
/katana/launch/katana_300_6m180.launch
</code></pre>

* Set the environment variables
Put the following in your .bashrc..
<pre><code>
cd
gedit .bashrc
</code></pre>
At the end of the file copy: 
<pre><code>
export KATANA_TYPE="katana_300_6m180"
source /opt/ros/hydro/setup.bash
source /home/ROS_katana/devel/setup.bash
</code></pre>
Save and close gedit. 
<pre><code>
source .bashrc
</code></pre>

* Launch the katana and start calibration
Open new terminal window and
<pre><code>
roslaunch katana katana.launch
</code></pre>

* Start the trajectory server and client
Open new terminal window and
<pre><code>
roslaunch katana_tutorials follow_joint_trajectory_client.launch
</code></pre>

* Send the start trajectory command 
Before running the following command make sure you are near the emergency stop button (or shutdown) and push it in case the Katana is headed for a collision.

Open new terminal window and
<pre><code>
rostopic pub /katana/start_trajectory std_msgs/Bool 1
</code></pre>
This should start the first trajectory with the Katana.

* Return the Katana to the initial position
Open new terminal window and
<pre><code>
rostopic pub /katana/start_trajectory2 std_msgs/Bool 1
</code></pre>
The Katana should now return to the initial (calibration) position.

Original readme: katana_driver
=============

This stack contains ROS hardware drivers, Gazebo plugins and other basic functionalities for the Neuronics Katana family of robot arms. Specifically, it provides: 

* JointTrajectory and FollowJointTrajectory execution on the physical arm (packages `katana`, `kni`, `katana_trajectory_filter`, `katana_msgs`),
* simulation of the Katana arm in Gazebo (packages `katana_gazebo_plugins`, `katana_arm_gazebo`),
* URDF descriptions (package `katana_description`),
* simple teleoperation (packages `katana_teleop`, `katana_joint_movement_adapter`), and
* some demo programs (package `katana_tutorials`).

For more information, visit the [katana_driver ROS wiki page](http://www.ros.org/wiki/katana_driver).
