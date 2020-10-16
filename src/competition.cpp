#include "competition.h"
#include "utils.h"

#include <std_srvs/Trigger.h>
////////////////////////////////////////////////////

Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
  node_ = node;
}

void Competition::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::ostringstream otopic;
    std::string topic;
//    part mypart;
    geometry_msgs::PoseStamped pose_target, pose_rel;
    if(msg->models.size() != 0){

        // ROS_INFO_STREAM("Camera_id : " << cam_idx);
        // ROS_INFO_STREAM("Logical camera: '" << msg->models.size() << "' objects.");
        int part_no = 0;
        ROS_INFO_STREAM("Parts detected by Logical camera " << cam_idx);
        ROS_INFO_STREAM(" ");
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

            mypart.type = msg->models[i].type;
            mypart.pose.position.x = tx;
            mypart.pose.position.y = ty;
            mypart.pose.position.z = tz;
            mypart.pose.orientation.x = pose_target.pose.orientation.x;
            mypart.pose.orientation.y = pose_target.pose.orientation.y;
            mypart.pose.orientation.z = pose_target.pose.orientation.z;
            // 3x3 Rotation matrix from quaternion
            tf2::Matrix3x3 m(q);

            // Roll Pitch and Yaw from rotation matrix
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Output the measure
            ROS_INFO("'%s' in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
                     topic.c_str(),
                     pose_target.header.frame_id.c_str(),
                     tx, ty, tz,
                     roll, pitch, yaw);
            std :: cout << " ------------------ ACCESSING EVERYTHING -----------------" << std :: endl;
            std :: cout << mypart.type << std :: endl;
            std :: cout << mypart.pose.position.x << std :: endl;
            std :: cout << mypart.pose.position.y << std :: endl;
            std :: cout << mypart.pose.position.z << std :: endl;
            std :: cout << mypart.pose.orientation.x << std :: endl;
            std :: cout << mypart.pose.orientation.y<< std :: endl;
            std :: cout << mypart.pose.orientation.z << std :: endl;
            std :: cout << "******************---------*******************------------***" << std :: endl;
        }
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM(" ");

    }

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


/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
  if (msg->data == "done" && competition_state_ != "done")
  {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

void Competition::order_callback(const nist_gear::Order::ConstPtr & msg) {
    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
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
