# Local path planner for NaviGANs - robot navigation in a socially compliant manner
This package implements a path following controller that manipulates a Husky robot to traverse a sequence of waypoints.



---


## Prerequisites
None.


###Additional dependencies
None.


---
## Installation and testing

Follow these steps to install and run the package:

1. Copy the folder 'navigans_ws' into your catkin workspace `src` folder. Make sure the permissions are set to `o+rwx` on your Python source files inside the directory `navigans_node/src`. 

2. In your `catkin_ws ($CATKIN)` folder, execute

     `$ catkin_make`

3. Source the environment for each terminal you work in. If necessary, add this line to your `.bashrc`

     `$CATKIN/devel/setup.bash`


5. To test the Action Server you should run the node first using a test image. To this end, we have provided a scritp and a launch file that will publish a test image as a ROS topic. Open a terminal and execute

    `$ roslaunch navigans_node navigans_path_basic.launch`

    If all goes well, you should see a message indicating ` [/navigans_husky1_control] initialized...`.  Also, you can use the command `rostopic list` to confirm that new actioni-related topics are published by this module.

6. To test the Action Client, open a new terminal and execute the following:

    `$ rosrun navigans_node simpleActionClient.py`
    
    If all goes well, you will see the Server in the first terminal reacting to the Client's request. Keep in mind that you may need to source the environment.
    