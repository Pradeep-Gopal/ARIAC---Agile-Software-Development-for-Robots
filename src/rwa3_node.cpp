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

#define MAX_NUMBER_OF_CAMERAS 17
std::array<std::array<part, 20>, 20>  parts_from_camera_main ;
std::vector<std::vector<std::vector<master_struct> > > master_vector_main (10,std::vector<std::vector<master_struct> >(10,std::vector <master_struct>(20)));


void during_kitting()
{
    for(int i=0; i < 10;  i++) {
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 20; k++) {
                if((master_vector_main[i][j][k].type == "pulley_part_red") || (master_vector_main[i][j][k].type == "pulley_part_blue") || (master_vector_main[i][j][k].type == "pulley_part_green")|| (master_vector_main[i][j][k].type == "disk_part_blue")|| (master_vector_main[i][j][k].type == "disk_part_red")|| (master_vector_main[i][j][k].type == "disk_part_green")|| (master_vector_main[i][j][k].type == "piston_part_blue")|| (master_vector_main[i][j][k].type == "piston_part_green")|| (master_vector_main[i][j][k].type == "piston_part_red")|| (master_vector_main[i][j][k].type == "gasket_part_blue")|| (master_vector_main[i][j][k].type == "gasket_part_red")|| (master_vector_main[i][j][k].type == "gasket_part_green"))
                {
                    ROS_INFO_STREAM(master_vector_main[i][j][k].type);
                }
            }
        }
    }
}

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

    parts_from_camera_main = comp.get_parts_from_camera();
    master_vector_main = comp.get_master_vector();

    during_kitting();

    gantry.goToPresetLocation(gantry.start_);
    gantry.goToPresetLocation(gantry.bin3_);

    //--You should receive the following information from a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
    part part_in_tray;
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.12;
    part_in_tray.pose.position.x = -0.2;
    part_in_tray.pose.position.x = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.0;
    part_in_tray.pose.orientation.w = 1.0;

    //--Go pick the part
    gantry.pickPart(my_part);
    //--Go place the part
    gantry.placePart(part_in_tray, "agv2");



    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}