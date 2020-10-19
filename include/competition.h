#ifndef COMPETITION_H
#define COMPETITION_H

#include <vector>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>

#include "utils.h"
# define MAX 40

using namespace std;
/**
 * @brief Competition class
 *
 */
class Competition
{
public:
    explicit Competition(ros::NodeHandle & node);
    void init();
    void startCompetition();
    void endCompetition();
    void PickUp();
    void fill_order();
    void competition_state_callback(const std_msgs::String::ConstPtr & msg);
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx);
    void print_parts_detected();
    void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg);
    void order_callback(const nist_gear::Order::ConstPtr & msg);
    double getClock();
    double getStartTime();
    std::string getCompetitionState();
    stats getStats(std::string function);
    //to printing vector
    void print_vec_string(vector<vector<vector<string> > >);
    void print_vec_double(vector<vector<vector<double> > >);
    //initializing all vector types
    vector<vector<vector<string> > > vec_type;
    vector<vector<vector<double> > > vec_position_x;
    vector<vector<vector<double> > > vec_position_y;
    vector<vector<vector<double> > > vec_position_z;
    vector<vector<vector<double> > > vec_orientation_x;
    vector<vector<vector<double> > > vec_orientation_y;
    vector<vector<vector<double> > > vec_orientation_z;
    vector<vector<vector<double> > > vec_orientation_w;
    //function to return the vectors to the main function
    std::vector<std::vector<std::vector<string> > > returnVecType();
    std::vector<std::vector<std::vector<double> > > returnVecPosX();
    std::vector<std::vector<std::vector<double> > > returnVecPosY();
    std::vector<std::vector<std::vector<double> > > returnVecPosZ();
    std::vector<std::vector<std::vector<double> > > returnVecOrientX();
    std::vector<std::vector<std::vector<double> > > returnVecOrientY();
    std::vector<std::vector<std::vector<double> > > returnVecOrientZ();
    std::vector<std::vector<std::vector<double> > > returnVecOrientW();

    std::array<std::array<part, 20>, 20> get_parts_from_camera();
private:
    ros::NodeHandle node_;

    std::string competition_state_;
    double current_score_;
    ros::Time competition_clock_;
    double competition_start_time_; // wall time in sec

    ros::Subscriber current_score_subscriber_;
    ros::Subscriber competition_state_subscriber_;
    ros::Subscriber competition_clock_subscriber_;
    ros::Subscriber orders_subscriber_;
    std::vector<nist_gear::Order> received_orders_;

    // to collect statistics
    stats init_;
};

#endif