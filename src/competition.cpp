#include "competition.h"
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include <vector>
using std::vector;
#include <string>
using namespace std;
int camera_no = 0;
//part parts_from_camera[40][40];
std::array<std::array<part, 20>, 20>  parts_from_camera ;
//array of structs
std :: array<part,20> struct_array;
////////////////////////////////////////////////////

Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
    node_ = node;
}

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


void Competition::fill_order() {
    int tot_order_size = 0;
    int tot_shipment_size = 0;
    int tot_prod_size = 0;
    std::cout << "received_orders_.size() " << received_orders_.size() << std::endl;
    for (int i = 0; i < received_orders_.size(); i++) {
        ROS_INFO_STREAM("FILL_ORDER FUNCTION CALLED --------------------------");
        ROS_INFO_STREAM(received_orders_[i]);
        ROS_INFO_STREAM("Number of Shipments --------------------------");
        tot_shipment_size = received_orders_[i].shipments.size();
        ROS_INFO_STREAM(tot_shipment_size);
        for (int j = 0; j < tot_shipment_size; j ++) {
            ROS_INFO_STREAM("Number of Products --------------------------");
            ROS_INFO_STREAM(received_orders_[i].shipments[j].products.size());
            tot_prod_size = received_orders_[i].shipments[j].products.size();
        }
    }
    tot_order_size = received_orders_.size();
    vector< vector< vector<string> > > vec_container(tot_order_size , vector< vector<string> > (tot_shipment_size, vector<string> (tot_prod_size) ) );
    // filling the vectors of orders, shipments and products
    for (int i = 0; i < tot_order_size; i++) {
        ROS_INFO_STREAM("Filling Orders ---------------- ----------");
        //fill in the vector here
        for (int j = 0; j < tot_shipment_size; j++) {
            ROS_INFO_STREAM("Filling Shipments --------------------------");
            //fill in the vector here
            for (int k = 0; k < tot_prod_size; k++) {
                ROS_INFO_STREAM("Filling Products --------------------------");
                //fill in the vector here
                vec_container[i][j][k] = received_orders_[i].shipments[j].products[k].type;
            }
        }
    }
    for (int i = 0; i < tot_order_size; i++) {
        ROS_INFO_STREAM("DISPLAYING ALL  Orders ---------------- ----------");
        for (int j = 0; j < tot_shipment_size; j++) {
            ROS_INFO_STREAM("DISPLAYING ALL Shipments --------------------------");
            for (int k = 0; k < tot_prod_size; k++) {
                ROS_INFO_STREAM("DISPLAYING ALL  Products --------------------------");
                ROS_INFO_STREAM(vec_container[i][j][k]);
            }
        }
    }
}

//        ROS_INFO_STREAM(received_orders_[i].products.size());
void Competition::print_parts_detected(){
    ROS_INFO_STREAM("------------------------------------------");

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
}

//void Competition::print_parts_detected(){
//    ROS_INFO_STREAM("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
//
//    for( auto &row : parts_from_camera)
//        for(auto &col : row)
//            ROS_INFO_STREAM(col.type);
//}

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

        // ROS_INFO_STREAM("Camera_id : " << cam_idx);
        // ROS_INFO_STREAM("Logical camera: '" << msg->models.size() << "' objects.");
        int part_no = 0;
//        ROS_INFO_STREAM("Parts detected by Logical camera " << cam_idx);
//        ROS_INFO_STREAM(" ");
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


//            parts_from_camera[i].type = msg->models[i].type;
//            parts_from_camera[i].pose.position.x = tx;
//            parts_from_camera[i].pose.position.y = ty;
//            parts_from_camera[i].pose.position.z = tz;
//            parts_from_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
//            parts_from_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
//            parts_from_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
//            parts_from_camera[i].pose.orientation.w = pose_target.pose.orientation.w;


            // Output the measure
//            ROS_INFO("'%s' in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//                     topic.c_str(),
//                     pose_target.header.frame_id.c_str(),
//                     tx, ty, tz,
//                     roll, pitch, yaw);

        }

//        ROS_INFO_STREAM(" ");
//        ROS_INFO_STREAM(" ");
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
    return competition_state_;
}