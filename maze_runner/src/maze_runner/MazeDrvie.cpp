// input: x, y of the detected obstacles
// output: speed, steering angle => motor msg
// written by team2
// 2023. 06. 05

#include "maze_runner/MazeDrive.hpp"

namespace Maze{

MazeDrive::MazeDrive()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);
    

    setParams(config);
    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriberLidar = mNodeHandler.subscribe(mSubscribedTopicNameLidar, mQueueSize, &MazeDrive::lidarCallback, this);
    mSubscriberUltra = mNodeHandler.subscribe(mSubscribedTopicNameUltra, mQueueSize, &MazeDrive::ultraCallback, this);

}

void MazeDrive::setParams(const YAML::Node& config)
{
    mSubscribedTopicNameLidar = config["TOPIC"]["SUB_NAME_LIDAR"].as<std::string>();
    mSubscribedTopicNameUltra = config["TOPIC"]["SUB_NAME_ULTRA"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<float>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<float>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<float>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<float>();
    mDecelerationStep_S = config["XYCAR"]["DECELERATION_STEP_S"].as<float>();

}

void MazeDrive::run()
{
    // variables
    float lidarD = 0.0;
    float theta = 0.0;
    float frontSafetyDistance = 30.0;

    float calculatedAngle = 0.0;

    // drive
    if (lidarD > frontSafetyDistance)
    {
        // output: calculatedAngle
        ultraDrive(ultraR, ultraL);
    }
    else
    {
        // output: calculatedAngle
        obstacleAvoidance(theta, lidarD);
    }
   
    float steeringAngle = std::max(static_cast<float>(-kXycarSteeringAangleLimit), std::min(static_cast<float>(calculatedAngle), static_cast<float>(kXycarSteeringAangleLimit)));

    speedControl(steeringAngle);
    drive(steeringAngle);
}


void MazeDrive::ultraDrive(float ultraR, float ultraL)
{
    // variable
    float ultraR = 0.0;
    float ultraL = 0.0;
    float sideSafetyDistance = 10.0;
    float calculatedAngle;

    if (ultraR > sideSafetyDistance && ultraL > sideSafetyDistance)
    {
        //go straight
        calculatedAngle = 0;
        return calculatedAngle;
    }

    else
    {
        if (ultraR <= sideSafetyDistance)
        {
            // turn left
            // return calculatedAngle
            // (maybe hardcoding...)
        }
        if (ultraL <= sideSafetyDistance)
        {
            // turn right
            // return calculatedAngle
            // (maybe hardcoding...)
        }
    }
}


void MazeDrive::obstacleAvoidance(float theta, float lidarD)
{
    // variable
    float distanceX = lidarD * cos(theta);
    float w = 30;       // car width (have to measure)
    float safetyFactor = 5;
    float calculatedAngle;

    if (distanceX >= w/2)
    {
        calculatedAngle = 0;
        return calculatedAngle;
    }
    else
    {
        float x = (w/2 + safetyFactor) / lidarD;
        calculatedAngle = theta - acos(x);
        return calculatedAngle;
    }
}



void MazeDrive::speedControl(float steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}


void MazeDrive::speedControl_straight(float steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep_S;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

void MazeDrive::drive(float steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    mPublisher.publish(motorMessage);
}

// output => lidarD, theta
void MazeDrive::lidarCallback(const sensor_msgs::LaserScan& message)
{
    float lidarIncrement = message.angle_increment;
    int32_t lidarRangeLimit = 505;
    int32_t lidarXrange = 128;
    
    // please check the function to get lidar data
    // i don't know how it works excectly
    for (int i = 0; i< lidarRangeLimit; i++)
    {
        float rad = i * lidarIncrement;
        float x = ran[i] * cos(rad);
        float y = -ran[i] * sin(rad);
        int pos_X_min = lidarXrange;
        int neg_X_max = -lidarXrange;

        lidarD[i] = x;
        theta[i] = y;
        
    }
}


// output: ultraR, ultraL
void MazeDrive::ultraCallback(const sensor_msgs::Int32MultiArray& message)
{
    
    // function to callback ultrasonic messages

}

} // namespace Maze
