#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <xycar_msgs/xycar_motor.h>
#include <cmath>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <yaml-cpp/yaml.h>


namespace Maze{
class MazeDrive{

public:
    using Ptr = std::unique_ptr<MazeDrive>;

    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    /**
     * @brief Construct a new maze drive object
     */
    MazeDrive();
    /**
     * @brief Run maze drive
     */
    void run();



private:
    void setParams(const YAML::Node& config);
    void lidarCallback(const sensor_msgs::LaserScan& message);
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& message_ultra);

    float ultraDrive();
    float obstacleAvoidance();

    void speedControl(float steeringAngle);
    void speedControl_straight(float steeringAngle);
    void drive_back(float steeringAngle);

    void drive(float steeringAngle);


    std::array<float,505> getResult_X() {return X;};
    std::array<float,505> getResult_Y() {return Y;};
    std::vector<float> getResult_Ran() {return ran;};
    std::vector<int> getResult_ultra() {return ultra_msg;};
    float getResult_theta() {return theta;};
    float getResult_lidarD() {return lidarD;};

private:

    //laser 
    std::array<float,505> X;
    std::array<float,505> Y;
    std::vector<float> ran;

    // ROX variables
    ros::NodeHandle mNodeHandler;
    ros::Publisher mPublisher;
    ros::Subscriber mSubscriberLidar;
    ros::Subscriber mSubscriberUltra;
    xycar_msgs::xycar_motor mMotorMessage;
    sensor_msgs::LaserScan mLidarMessage;
    std_msgs::Int32MultiArray mUltraMessage;
    std::string mPublishingTopicName;
    std::string mSubscribedTopicNameLidar;
    std::string mSubscribedTopicNameUltra;
    uint32_t mQueueSize;


    // Xycar Device variables
    float mXycarSpeed;                 ///< Current speed of xycar
    float mXycarMaxSpeed;              ///< Max speed of xycar
    float mXycarMinSpeed;              ///< Min speed of xycar
    float mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    float mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    float mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold
    float mDecelerationStep_S;           ///< How much would deaccelrate xycar depending on threshold

    std::array<float,505> lidarX;
    std::array<float,505> lidarY;
    std::vector<float> dist;
    std::vector<int> ultra_msg;
    std::vector<int> ultra_array;

    float theta;
    float lidarD;
    float lidarIncrement;
};

}