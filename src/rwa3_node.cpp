// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>
using namespace std;
#define MAX_NUMBER_OF_CAMERAS 17
std::array<std::array<part, 20>, 20>  parts_from_camera_main ;
//hard coding the x,y and z dimensions of the 3D vector, which can be obtained from the class
//through getter functions. -- not hard.
std::vector<std::vector<std::vector<string> > > returned_vec_string_TYPE (1,std::vector<std::vector<string> >(1,std::vector <string>(4)));
std::vector<std::vector<std::vector<string> > > returned_vec_string (1,std::vector<std::vector<string> >(1,std::vector <string>(4)));
std::vector<std::vector<std::vector<double> > > returned_vec_double (1,std::vector<std::vector<double> >(1,std::vector <double>(4)));
int main(int argc, char ** argv) {

    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);

    //Array of Logical Camera Subscribers
    ros::Subscriber logical_camera_subscriber_ [MAX_NUMBER_OF_CAMERAS];
    std::ostringstream otopic;
    std::string topic;

    for (int idx = 0; idx < MAX_NUMBER_OF_CAMERAS; idx++){
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/logical_camera_" << idx;
        topic = otopic.str();
        logical_camera_subscriber_[idx] = node.subscribe<nist_gear::LogicalCameraImage>
                (topic, 10, boost::bind(&Competition::logical_camera_callback, &comp, _1, idx));
    }
    comp.init();
    std::string c_state = comp.getCompetitionState();
    comp.getClock();
    GantryControl gantry(node);
    gantry.init();
    returned_vec_string_TYPE = comp.returnVecType();
    comp.print_vec_string(returned_vec_string_TYPE);//for printing type, which is of string data-type
    returned_vec_double = comp.returnVecPosX();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecPosY();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecPosZ();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecOrientX();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecOrientY();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecOrientZ();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    returned_vec_double = comp.returnVecOrientW();
    comp.print_vec_double(returned_vec_double);//for printing double data-type vectors
    ROS_INFO_STREAM("------- Going to Preset Locations -------");
    //going to preset locations
//    gantry.goToPresetLocation(gantry.start_);
//    gantry.goToPresetLocation(gantry.bin13_);
//    gantry.goToPresetLocation(gantry.bin16_);
    //next three are waypoints before you reach shelf
//    gantry.goToPresetLocation(gantry.start_);
//    gantry.goToPresetLocation(gantry.waypoint_1_);
//    gantry.goToPresetLocation(gantry.waypoint_2_);
//    gantry.goToPresetLocation(gantry.shelf5_);
    ROS_INFO_STREAM("PRINTING VECTOR ELEMENTS ONLY");
    for(int i = 0;i<1;i++){
        for(int j=0;j<1;j++) {
            for (int k = 0; k < 4; k++) {
                ROS_INFO_STREAM(returned_vec_string_TYPE[i][j][k]);
            }
        }
    }
    ROS_INFO_STREAM("--------------------------------for loop starts now ----------------------------------");
    parts_from_camera_main = comp.get_parts_from_camera(); //this is a 20 by 20 - 2 dimensional vector
    string part_to_find;
    string s;
    int before_removal = 0;
    int after_removal = 0;
    int diff_vec_size = 0;
    int vec_idx=0;
    bool vec_flag = false; //boolean to check if all the orders have been completed
    do{
    for(int i = 0;i<=20;i++){
        for(int j=0;j<=20;j++){
            string line = parts_from_camera_main[i][j].type;
            if (!line.empty()) {
                diff_vec_size=0; //resetting the value
                part_to_find = parts_from_camera_main[i][j].type;
                ROS_INFO_STREAM("The part from the logical camera detected : "<<part_to_find);
                before_removal = returned_vec_string_TYPE[0][0].size();
                if (before_removal == 0){
                    ROS_INFO_STREAM("XXXXXXXXXXXXXXXX=== BREAK OUT OF THE LOOP ===XXXXXXXXXXXXXXXX");
                    vec_flag; // should break out of the loop
                    break;
                }
                ROS_INFO_STREAM("before_removal the order had > "<<before_removal);
                ROS_INFO_STREAM("VECTOR BEFORE REMOVAL");
                ROS_INFO_STREAM("<<<<<<begin>>>>>>");
                for(vec_idx=0; vec_idx < returned_vec_string_TYPE[0][0].size(); vec_idx++){
                    //printing vector before removal of the parts
                    ROS_INFO_STREAM(returned_vec_string_TYPE[0][0].at(vec_idx));
                }
                ROS_INFO_STREAM("<<<<<<end>>>>>>");
                returned_vec_string_TYPE[0][0].erase(std::remove(returned_vec_string_TYPE[0][0].begin(), returned_vec_string_TYPE[0][0].end(), part_to_find), returned_vec_string_TYPE[0][0].end());
                after_removal = returned_vec_string_TYPE[0][0].size();
                ROS_INFO_STREAM("after_removal the order has > "<<after_removal);
                ROS_INFO_STREAM("VECTOR AFTER REMOVAL");
                ROS_INFO_STREAM("<<<<<<begin>>>>>>");
                for(vec_idx=0; vec_idx < returned_vec_string_TYPE[0][0].size(); vec_idx++){
                    //printing vector after removal of the parts
                    ROS_INFO_STREAM(returned_vec_string_TYPE[0][0].at(vec_idx));
                }
                ROS_INFO_STREAM("<<<<<<end>>>>>>");
                //checking if all orders have been completed
                diff_vec_size = abs(before_removal - after_removal);
                ROS_INFO_STREAM("diff_vec_size");
                ROS_INFO_STREAM(diff_vec_size);
                if (diff_vec_size !=1){
                    ROS_INFO_STREAM("TWO PARTS REMOVED");
                    for(int keep_insert;keep_insert<diff_vec_size-1;keep_insert++){
                        returned_vec_string_TYPE[0][0].push_back(part_to_find);
                    }
                    if (part_to_find=="pulley_part_red"){
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>GOING TO SHELF 15");
                        gantry.goToPresetLocation(gantry.start_);
                        ROS_INFO_STREAM("Start location reached");
                        gantry.goToPresetLocation(gantry.waypoint_1_);
                        ROS_INFO_STREAM("waypont1 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_2_);
                        ROS_INFO_STREAM("waypoint2 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_3_);
                        ROS_INFO_STREAM("waypoint3 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_4_);
                        ROS_INFO_STREAM("waypoint4 location reached");

                        gantry.pickPart(parts_from_camera_main[i][j]);
                        ROS_INFO_STREAM("Part picked");

                        gantry.goToPresetLocation(gantry.waypoint_4_);
                        ROS_INFO_STREAM("waypoint4 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_3_);
                        ROS_INFO_STREAM("waypoint3 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_2_);
                        ROS_INFO_STREAM("waypoint2 location reached");
                        gantry.goToPresetLocation(gantry.waypoint_1_);
                        ROS_INFO_STREAM("waypoint1 location reached");
                        gantry.goToPresetLocation(gantry.start_);
                    }
                }
                else if (diff_vec_size ==1) {
                    ROS_INFO_STREAM("ONE PART REMOVED");
                    if(part_to_find=="disk_part_blue"){
                        //go to bin 13
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>GOING TO BIN 13");
                        gantry.goToPresetLocation(gantry.start_);
                        gantry.goToPresetLocation(gantry.bin13_);
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>picking up the blue part");
                        gantry.pickPart(parts_from_camera_main[i][j]);
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>going back to the start position");
                        gantry.goToPresetLocation(gantry.start_);
                        gantry.deactivateGripper("left_arm");
                        ROS_INFO_STREAM("Start location reached");

                    }
                    else if(part_to_find=="disk_part_green"){
                        //go to bin 16
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>GOING TO BIN 16");
                        gantry.goToPresetLocation(gantry.start_);
                        gantry.goToPresetLocation(gantry.bin16_);
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>picking up the blue part");
                        gantry.pickPart(parts_from_camera_main[i][j]);
                        ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>going back to the start position");
                        gantry.goToPresetLocation(gantry.start_);
                        ROS_INFO_STREAM("Start location reached");
                        gantry.deactivateGripper("left_arm");
                    }
                }
                else{
                    ROS_INFO_STREAM("THERE WAS NOTHING TO REMOVE");
                }

            }//check if line received from the 2 dimensional array is empty
        }// end of the inner for loop with j
    }//end of the outer for loop with i
    }while (!vec_flag);//end of the do-while loop
    ROS_INFO_STREAM("ALL DONE"); // out of all the loops
    //--You shold receive the following information from a camera
//    part my_part;
//    my_part.type = "pulley_part_red";
//    my_part.pose.position.x = 4.365789;
//    my_part.pose.position.y = 1.173381;
//    my_part.pose.position.z = 0.728011;
//    my_part.pose.orientation.x = 0.012;
//    my_part.pose.orientation.y = -0.004;
//    my_part.pose.orientation.z = 0.002;
//    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
//    part part_in_tray;
//    part_in_tray.type = "pulley_part_red";
//    part_in_tray.pose.position.x = -0.12;
//    part_in_tray.pose.position.x = -0.2;
//    part_in_tray.pose.position.x = 0.0;
//    part_in_tray.pose.orientation.x = 0.0;
//    part_in_tray.pose.orientation.y = 0.0;
//    part_in_tray.pose.orientation.z = 0.0;
//    part_in_tray.pose.orientation.w = 1.0;

    //--Go pick the part
//    gantry.pickPart(my_part);
    //--Go place the part
//    gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}