#include "competition.h"
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include "gantry_control.h"
#include "../include/competition.h"

#include <vector>
using std::vector;
#include <string>
using namespace std;
int camera_no = 0;
//part parts_from_camera[40][40];
std::array<std::array<part, 20>, 20>  parts_from_camera ;
//array of structs
std :: array<part,20> struct_array;

int test_dimensions = 3;
int tot_order_size = 0;
int tot_shipment_size = 0;
int temp_tot_shipment_size = 0;
int tot_prod_size = 0;
int temp_tot_prod_size = 0;

////////////////////////////////////////////////////

Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
    node_ = node;
}
//VECTOR PRINT FUNC
 void Competition::print_vec_string(vector<vector<vector<string> > > vec_print){
     ROS_INFO_STREAM("PRINTING STRING TYPE VEC");
     for (int i = 0; i < tot_order_size; i++) {
         for (int j = 0; j < tot_shipment_size; j++) {
             for (int k = 0; k < tot_prod_size; k++) {
                 ROS_INFO_STREAM("PRINTED part is :>> "<<vec_print[i][j][k]);
             }
         }
     }
}

void Competition::print_vec_double(vector<vector<vector<double> > > vec_print){
    ROS_INFO_STREAM("PRINTING DOUBLE TYPE VEC");
    for (int i = 0; i < tot_order_size; i++) {
        for (int j = 0; j < tot_shipment_size; j++) {
            for (int k = 0; k < tot_prod_size; k++) {
                ROS_INFO_STREAM("part character "<<vec_print[i][j][k]);
            }
        }
    }
}
//-----


//ALL RETURN VEC FUNCTIONS
std::vector<std::vector<std::vector<string> > > Competition::returnVecType()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_type;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecPosX()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_position_x;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecPosY()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_position_y;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecPosZ()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_position_z;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecOrientX()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_orientation_x;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecOrientY()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_orientation_y;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecOrientZ()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_orientation_z;
}
std::vector<std::vector<std::vector<double> > > Competition::returnVecOrientW()
{
    ROS_INFO_STREAM("returning vec_type to rwa3_node.cpp");
    return vec_orientation_w;
}
//------

void Competition::init() {
    // Subscribe to the '/ariac/current_score' topic.
    double time_called = ros::Time::now().toSec();
    competition_start_time_ = ros::Time::now().toSec();

    // Subscribe to the '/ariac/competition_state' topic.
    ROS_INFO("Subscribe to the /ariac/competition_state topic...");
    competition_state_subscriber_ = node_.subscribe(
            "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

    // Subscribe to the '/clock' topic.
    ROS_INFO("Subscribe to the /clock...");
    competition_clock_subscriber_ = node_.subscribe(
            "/clock", 10, &Competition::competition_clock_callback, this);

    ROS_INFO("Subscribe to the /orders...");
    orders_subscriber_ = node_.subscribe(
            "/ariac/orders", 10, &Competition::order_callback, this);

    startCompetition();

    init_.total_time += ros::Time::now().toSec() - time_called;

}

void Competition::PickUp(){

}
void Competition::fill_order() {

    std::cout << "received_orders_.size() " << received_orders_.size() << std::endl;
    for (int i = 0; i < received_orders_.size(); i++) {
        ROS_INFO_STREAM("FILL_ORDER FUNCTION CALLED --------------------------");
        ROS_INFO_STREAM(received_orders_[i]);
        ROS_INFO_STREAM("Number of Shipments --------------------------");
        temp_tot_shipment_size = received_orders_[i].shipments.size();
        //rewriting the smaller value with the bigger value
        if (temp_tot_shipment_size>tot_shipment_size){
            tot_shipment_size = temp_tot_shipment_size;
        }
        ROS_INFO_STREAM(tot_shipment_size);
        for (int j = 0; j < tot_shipment_size; j ++) {
            ROS_INFO_STREAM("Number of Products --------------------------");
            ROS_INFO_STREAM(received_orders_[i].shipments[j].products.size());
            temp_tot_prod_size = received_orders_[i].shipments[j].products.size();
            //rewriting the smaller value with the bigger value
            if (temp_tot_prod_size>tot_prod_size){
                tot_prod_size = temp_tot_prod_size;
            }
        }
    }
    tot_order_size = received_orders_.size();
    for(int i=0; i<tot_order_size; i++)
    {
        vec_type.push_back(vector<vector<string> >()); //initialize the first index with a 2D vector
        for(int j=0; j<tot_shipment_size; j++)
        {
            vec_type[i].push_back(vector<string>()); //initialize the 2 index with a row of strings
            for(int k=0; k<tot_prod_size; k++)
            {
                vec_type[i][j].push_back("dummy");
            }
        }
    }

    for(int i=0; i<tot_order_size; i++)
    {
        vec_position_x.push_back(vector<vector<double> >()); //initialize the first index with a 2D vector
        vec_position_y.push_back(vector<vector<double> >());
        vec_position_z.push_back(vector<vector<double> >());
        vec_orientation_x.push_back(vector<vector<double> >());
        vec_orientation_y.push_back(vector<vector<double> >());
        vec_orientation_z.push_back(vector<vector<double> >());
        vec_orientation_w.push_back(vector<vector<double> >());
        for(int j=0; j<tot_shipment_size; j++)
        {
            vec_position_x[i].push_back(vector<double>()); //initialize the 2 index with a row of strings
            vec_position_y[i].push_back(vector<double>());
            vec_position_z[i].push_back(vector<double>());
            vec_orientation_x[i].push_back(vector<double>());
            vec_orientation_y[i].push_back(vector<double>());
            vec_orientation_z[i].push_back(vector<double>());
            vec_orientation_w[i].push_back(vector<double>());
            for(int k=0; k<tot_prod_size; k++)
            {
                vec_position_x[i][j].push_back(0.0);
                vec_position_y[i][j].push_back(0.0);
                vec_position_z[i][j].push_back(0.0);
                vec_orientation_x[i][j].push_back(0.0);
                vec_orientation_y[i][j].push_back(0.0);
                vec_orientation_z[i][j].push_back(0.0);
                vec_orientation_w[i][j].push_back(0.0);
            }
        }
    }


    ROS_INFO_STREAM("NEW VECTOR SIZES ::::-----------");
    //just for checking purposes, doesn't do anything
    ROS_INFO_STREAM("tot_order_size : "<<tot_order_size);
//    ROS_INFO_STREAM(vec_type.size());
    ROS_INFO_STREAM("tot_shipment_size : "<<tot_shipment_size);
//    ROS_INFO_STREAM(vec_type[0].size());
    ROS_INFO_STREAM("tot_prod_size : "<<tot_prod_size);
//    ROS_INFO_STREAM(vec_type[0][0].size());
    ROS_INFO_STREAM("completed--------------------------");

    // filling the vectors of orders, shipments and products
    for (int i = 0; i < tot_order_size; i++) {
        ROS_INFO_STREAM("Filling Orders ---------------");
        //fill in the vector here
        for (int j = 0; j < tot_shipment_size; j++) {
            ROS_INFO_STREAM("Filling Shipments ------------------");
            //fill in the vector here
            for (int k = 0; k < tot_prod_size; k++) {
                ROS_INFO_STREAM("Filling Products --------------------------");
                //fill in the vector here
                vec_type[i][j][k] = received_orders_[i].shipments[j].products[k].type;
                vec_position_x[i][j][k] = received_orders_[i].shipments[j].products[k].pose.position.x;;
                vec_position_y[i][j][k] = received_orders_[i].shipments[j].products[k].pose.position.y;;
                vec_position_z[i][j][k] = received_orders_[i].shipments[j].products[k].pose.position.z;;
                vec_orientation_x[i][j][k] = received_orders_[i].shipments[j].products[k].pose.orientation.x;;;
                vec_orientation_y[i][j][k] = received_orders_[i].shipments[j].products[k].pose.orientation.y;;;
                vec_orientation_z[i][j][k] = received_orders_[i].shipments[j].products[k].pose.orientation.z;;;
                vec_orientation_w[i][j][k] = received_orders_[i].shipments[j].products[k].pose.orientation.w;;;
            }
        }
    }

//    ROS_INFO_STREAM(" ------- The elements inside the vector ------- ");
//    for (int i = 0; i < tot_order_size; i++) {
//        for (int j = 0; j < tot_shipment_size; j++) {
//            for (int k = 0; k < tot_prod_size; k++) {
//                ROS_INFO_STREAM(" vec_type printed values " << vec_type[i][j][k]);
//            }
//        }
//    }

}

void Competition::print_parts_detected(){
    ROS_INFO_STREAM("< | < | < | parts detected > | > | > | >");

    for (int i = 0; i < parts_from_camera.size(); i++)
    {
        std::cout << "parts from camera = " << i << std::endl;
        std::cout << std::endl;
        for (int j = 0; j < parts_from_camera[i].size(); j++){
            std::cout << " " << parts_from_camera[i][j].type;
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }
    ROS_INFO_STREAM("< | < | < | ------ X ------ > | > | > | >");
}

void Competition::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::ostringstream otopic;
    std::string topic;
    std::ostringstream otopic_part;
    std::string topic_part;

    geometry_msgs::PoseStamped pose_target, pose_rel;
    if(msg->models.size() != 0){
        int part_no = 0;
        for(int i = 0; i<msg->models.size(); i++)
        {
            part_no++;
            otopic.str("");
            otopic.clear();
            otopic << "logical_camera_" << cam_idx << "_" << msg->models[i].type<< "_frame";
            topic = otopic.str();
            // ROS_INFO_STREAM(topic);
            ros::Duration timeout(5.0);
            geometry_msgs::TransformStamped transformStamped;
            pose_rel.header.frame_id = "logical_camera_" + std::to_string(cam_idx) + "_frame";
            pose_rel.pose = msg->models[i].pose;

            try{
                transformStamped = tfBuffer.lookupTransform("world", pose_rel.header.frame_id,
                                                            ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            tf2::doTransform(pose_rel, pose_target, transformStamped);
            // ROS_INFO_STREAM("Camera coordinates of " << topic << " no of parts - " << msg->models.size());
            // ROS_INFO_STREAM(pose_target);

            double tx = pose_target.pose.position.x;
            double ty = pose_target.pose.position.y;
            double tz = pose_target.pose.position.z;

            // Orientation quaternion
            tf2::Quaternion q(
                    pose_target.pose.orientation.x,
                    pose_target.pose.orientation.y,
                    pose_target.pose.orientation.z,
                    pose_target.pose.orientation.w);

            // 3x3 Rotation matrix from quaternion
            tf2::Matrix3x3 m(q);

            // Roll Pitch and Yaw from rotation matrix
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            otopic_part.str("");
            otopic_part.clear();
            otopic_part << msg->models[i].type << "_" << cam_idx << "_" << part_no;
            topic_part = otopic_part.str();

            parts_from_camera[cam_idx][i].type = msg->models[i].type;
            parts_from_camera[cam_idx][i].pose.position.x = tx;
            parts_from_camera[cam_idx][i].pose.position.y = ty;
            parts_from_camera[cam_idx][i].pose.position.z = tz;
            parts_from_camera[cam_idx][i].pose.orientation.x = pose_target.pose.orientation.x;
            parts_from_camera[cam_idx][i].pose.orientation.y = pose_target.pose.orientation.y;
            parts_from_camera[cam_idx][i].pose.orientation.z = pose_target.pose.orientation.z;
            parts_from_camera[cam_idx][i].pose.orientation.w = pose_target.pose.orientation.w;

        }
    }
}


/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
        ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
}

void Competition::order_callback(const nist_gear::Order::ConstPtr & msg) {
//    ROS_INFO_STREAM("Received order:\n" << *msg);

    received_orders_.push_back(*msg);
    Competition::fill_order();
}

/// Called when a new message is received.
void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg) {
    competition_clock_ = msg->clock;
}


void Competition::startCompetition() {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists()) {
        ROS_INFO("[competition][startCompetition] Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("[competition][startCompetition] Competition is now ready.");
    }
    ROS_INFO("[competition][startCompetition] Requesting competition start...");
    std_srvs::Trigger srv;
    start_client.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
    } else {
        ROS_INFO("[competition][startCompetition] Competition started!");
    }
}


void Competition::endCompetition() {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient end_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!end_client.exists()) {
        ROS_INFO("[competition][endCompetition] Waiting for the end_competition to be ready...");
        end_client.waitForExistence();
        ROS_INFO("[competition][endCompetition] end_competition is now ready.");
    }
    ROS_INFO("[competition][endCompetition] Requesting competition end...");
    std_srvs::Trigger srv;
    end_client.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("[competition][endCompetition] Failed to end the competition: " << srv.response.message);
    } else {
        ROS_INFO("[competition][endCompetition] Competition ended!");
    }
}


stats Competition::getStats(std::string function) {
    if (function == "init") return init_;

}

double Competition::getStartTime() {
    return competition_start_time_;
}

double Competition::getClock() {
    double time_spent = competition_clock_.toSec();
    ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
    return time_spent;
}


std::string Competition::getCompetitionState() {
    ROS_INFO_STREAM("------------ competition_state_ BELOW -----------");
    return competition_state_;
}


std::array<std::array<part, 20>, 20> Competition::get_parts_from_camera()
{
    return parts_from_camera;
}
