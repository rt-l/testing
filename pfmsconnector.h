#ifndef PIPES_H
#define PIPES_H
#include <string>
#include <vector>

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>

#include "pfms_types.h"

////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
//#include "tf/transform_datatypes.h"

using std::string;

class PfmsConnector
{
public:


    /*! @brief Default constructor
     *
     *  Will establish the communication to ROS
     *  Should ONLY be created to establish communication and reused
     */
    PfmsConnector();

    ~PfmsConnector();

    /*! @brief Send Ackerman command to the pipe, non-blocking call
     * Will only send steering if it has changed from previous value.
     *
     *  @param cmd - the command to be sent
     */
    void send(pfms::commands::Ackerman cmd);

    /*! @brief Send Quadcopter command to the pipe, non-blocking call
     *
     *  @param cmd - the command to be written
     */
    void send(pfms::commands::Quadcopter cmd);

    /*! @brief Send SkidSteer command to the pipe, non-blocking call
     *
     *  @param cmd - the command to be written
     */
    void send(pfms::commands::SkidSteer cmd);

    /*! @brief Send Goal to the pipe for visualisation, non-blocking call
     *
     *  @param goal - the goal to be written to pipe
     */
    void send(pfms::geometry_msgs::Goal goal);

    /*! @brief Send status to the pipe, non-blocking call
     *
     *  @param status - Only responds to UAV status, actions only take-off and landing status
     */
    void send(pfms::PlatformStatus status);

    /*! @brief Reads LaserScan from the pipe, blocking call, will
     * be suspended until read
     *
     *  @param laserScan - the LaserScan from the pipe 
     *  @return true indicates is command written sucsesfully,
     * false timeout occured (pipe might not be open)
     */
    bool read(pfms::sensor_msgs::LaserScan & laserScan);

    /*! @brief Reads Sonar from the pipe, blocking call, will
     * be suspended until data written or timeout (3s)
     *
     *  @param scan - the Sonar reading from the pipe (read/send)
     *  @return true indicates is command written sucsesfully,
     * false timeout occured (pipe might not be open)
     */
    bool read(pfms::sensor_msgs::Sonar & scan);

    /*! @brief Reads Odometry of platforms from the pipe, depending on type
     *  blocking call, will be suspended until commend written or timeout (3s)
     *
     *  @param odo - the Odometry from the pipe (read)
     *  @param type - the Platform type 
     *  @return true indicates command written sucsesfully,
     * false timeout occured (pipe might not be open)
     */
    bool read(pfms::nav_msgs::Odometry& odo,pfms::PlatformType type);

private:
    bool readAckermanOdometry(pfms::nav_msgs::Odometry& msg);
    bool readSkidSteerOdometry(pfms::nav_msgs::Odometry& msg);
    bool readQuadcopterOdometry(pfms::nav_msgs::Odometry& msg);
    void ackermanOdoCallback(const nav_msgs::msg::Odometry& msg);
    void skidSteerOdoCallback(const nav_msgs::msg::Odometry& msg);
    void quadcopterOdoCallback(const geometry_msgs::msg::Pose& msg);
    void laserCallback(const sensor_msgs::msg::LaserScan& msg);
    void sonarCallback(const sensor_msgs::msg::Range& msg);
    void spin(void);
    void addMarker(double x, double y, unsigned int seq);
    void quadcopterLand(void);
    void quadcopterTakeOff(void);

private:

    struct sync_details{
        std::condition_variable cv; //<! convar to synch getting and reading data
        std::atomic<bool> ready;    //<! Indicates if new  data has been recieved
        std::mutex mtx;             //<! mutex to protect data
    };

    pfms::nav_msgs::Odometry ackermanOdo_; //<! Local record of odometry;
    pfms::nav_msgs::Odometry skidSteerOdo_; //<! Local record of odometry;
    pfms::nav_msgs::Odometry quadcopterOdo_; //<! Local record of odometry;

    pfms::sensor_msgs::Sonar sonar_;
    pfms::sensor_msgs::LaserScan laser_;
    std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in destructor
    std::atomic<bool> running_; // We use this to indicate threads shoudl still be running

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ackermanOdoSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr skidSteerOdoSub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr quadcopterOdoSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonarSub_;

    sync_details ackermanOdoSync_;
    sync_details skidSteerOdoSync_;
    sync_details quadcopterOdoSync_;
    sync_details sonarSync_;
    sync_details laserSync_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vizPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr quadcopterCmdPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr skidSteerCmdPub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr quadcopterTakeOffPub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr quadcopterLandPub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttlePub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brakePub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steeringPub_;

    rclcpp::Node::SharedPtr node_;

    int marker_counter_;
    unsigned int seq_;
    visualization_msgs::msg::MarkerArray marker_array_;    

    bool quadcopterFlying_;
    unsigned int ugvSeq_;
    // double ugvSteering_;

    std::chrono::seconds timeout;

};

#endif // PIPES_H
