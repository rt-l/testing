#ifndef PFMS_COMMANDER_H
#define PFMS_COMMANDER_H

#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <string>
#include <future>

#include "pfms_types.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class PfmsHog
{
public:
    PfmsHog(pfms::PlatformType platform);
    PfmsHog(pfms::PlatformType platform, bool debug);
    ~PfmsHog();

    std::string mapServiceToPlatform(pfms::PlatformType platform){
        std::string service;
        switch(platform) {
          case pfms::PlatformType::ACKERMAN:
            service = "audi_check_goals";
            break;
          case pfms::PlatformType::SKIDSTEER:
            service = "husky_check_goals";
            break;
          case pfms::PlatformType::QUADCOPTER:
            service = "drone_check_goals";
            break;
        }
        return service;
    }

    std::string mapModelNameToPlatform(pfms::PlatformType platform){
        std::string frame;
        switch(platform) {
          case pfms::PlatformType::ACKERMAN:
            frame = "orange";
            break;
          case pfms::PlatformType::SKIDSTEER:
            frame = "husky";
            break;
          case pfms::PlatformType::QUADCOPTER:
            frame = "drone";
            break;
        }
        return frame;
    }    

    /*! @brief Teleport the platform to the location specified in the odometry, the change is instateneous.
     *
     *  @param odo - The odometry, currently yaw is the only orientation used, the velocity supplied is ignored (and will be set to zero)
     */
    bool teleport(pfms::nav_msgs::Odometry odo);

    /*! @brief Specify the anticipated goal, the platform  will need to travel through this goal 
     *
     *  @param goal - The goal (point) that needs to be reached
     *  @return goal was reached by platform.
     *  @note Internally the code uses ROS services and another ROS component to check goal has been reached by platform
     */
    bool setGoal(pfms::geometry_msgs::Point goal);

    /*! @brief Specify the anticipated goals, the platform will need to travel through all of these goals in the exact order supplied
     *
     *  @param goals - The goals (points) that needs to be reached, the vector implies the order (from 0th to last element)
     *  @return goals were reached by platform in the exact order supplied
     */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);


    // /*! @brief Send the goals to be visualised on rviz
    //  *
    //  *  @param goals - The points that needs to be reached
    //  *  @param platform - platform (will change colour for platform type)
    //  */
    // bool visualiseGoals(std::vector<pfms::geometry_msgs::Point> goals,pfms::PlatformType platform);

    /*! @brief Check that goals set via @sa writeCommand has been travelled through,
     * checks this via a service call to gazebo_tf odo checker
     *
     *  @param [in|out] - closest distances to goals supplied
     *  @return all goals were travelled through in supplied order
     */
    bool checkGoalsReached(std::vector<double> &distances);

    // /*! @brief Check the pose of the frame
    //  * checks this via a service call to gazebo
    //  *
    //  *  @param frame - frame name for ros to check with reference to world frame
    //  *  @return odometry - the pose of the frame with reference to world frame
    //  */
    // pfms::nav_msgs::Odometry checkTransform(std::string frame);

private:
    void spin(void);

    /*! @brief Obtain MarkerArray of CUBES from geometry_msgs::Point
    * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
    *
    *  @param goals - vector of geometry_msgs::Point
    *  @return
    */
    visualization_msgs::msg::MarkerArray produceMarkerList(std::vector<pfms::geometry_msgs::Point> goals,std_msgs::msg::ColorRGBA color);

    // std::string replaceAll(std::string str, const std::string &from, const std::string &to);

private:

    struct sync_details{
        std::condition_variable cv; //<! convar to synch getting and reading data
        std::atomic<bool> ready;    //<! Indicates if new  data has been recieved
        std::mutex mtx;             //<! mutex to protect data
    };


    pfms::PlatformType platform_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr clientParams_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr clientDelete_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr clientSpawn_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clientGoals_;     

    void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    void responseParam_callback(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future);
    void responseDelete_callback(rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future);
    void responseSpawn_callback(rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future);
    bool service_done_ = false; // inspired from action client c++ code

    bool debug_;/*!< bool indicating if debug information to be provided */
    // ros::Publisher viz_pub_; //<! Publisher to rviz
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uavCmdPub_;//<! Publisher to stop quadcopter
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goalPub_;//<! Publisher to stop quadcopter
    std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in destructor
    std::atomic<bool> running_; // We use this to indicate threads shoudl still be running
    unsigned int markerCount_;

    bool goalSuccess_;
    std::vector<double> distances_;
    std::atomic<bool> checkedGoals_; // We use this to indicate threads shoudl still be running

    bool serviceTeleport_done_ = false; // inspired from action client c++ code
    std::atomic<bool> paramDone_ ;
    
    bool serviceDelete_done_ = false; // inspired from action client c++ code
    std::atomic<bool> deleteDone_ ;

    bool serviceSpawn_done_ = false; // inspired from action client c++ code
    std::atomic<bool> spawnDone_ ;

    std::string robotDescription_;

};

#endif // PfmsHog_H
