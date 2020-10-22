# RWA_3

## Team members
1. Pradeep Gopal
2. Rajesh 
3. Govind
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

1. Copy and paste the package in the /ariac_ws/src/rwa3_group1 directory
2. Open a terminal and type the following commands
3. cd /ariac_ws
4. catkin build rwa3_group1
5. source devel/setup.bash
6. roslaunch rwa3_group1 rwa3.launch load_moveit:=true

Wait till the terminal says "you can start planning now"

7. Open a new terminal and enter the following command to run the node.
8. cd /ariac_ws
9. source devel/setup.bash
10. rosrun rwa3_group1 rwa3_node
