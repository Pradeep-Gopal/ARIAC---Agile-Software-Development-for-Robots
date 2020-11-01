# ARIAC Competition

Picking and delivering parts in an industrial environment with faulty parts

## Results
[Scenario 1](https://www.youtube.com/watch?v=Xu1gFQL5WeM&list=PL_HqcgW4roXofxZxdxJUef4rXWphtUOpH&index=1)
[Scenario 2](https://www.youtube.com/watch?v=MJz2P6-rJoY&list=PL_HqcgW4roXofxZxdxJUef4rXWphtUOpH&index=2)
[Scenario 2](https://youtu.be/T9MkpMUNcMY)

## Team members
1. Pradeep Gopal
2. Rajesh 
3. Govind
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

1. git clone the package in the /ariac_ws/src/ directory and rename the package to rwa3_group1
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
