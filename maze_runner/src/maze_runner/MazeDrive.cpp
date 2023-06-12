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

    float min_can = 0.6;
    //float min_ran = *min_element(ran.begin(), ran.end(),myfn);
    int min_idx = 0;  //min_element(ran.begin(),ran.end(),myfn)-ran.begin();
    int ss = ran.size();
    for (int i = 0; i < ss; i++)
    {

    //    std::cout << "ran : " << ran[i] << std::endl;

        if ((ran[i]>0.2))
        {
            if (ran[i] < min_can){
                min_idx = i;
                min_can = ran[i];
            }
        }
    }
    //std::cout << "hheheeh" << min_can << std::endl;
    theta = (min_idx-ss/2) * lidarIncrement;
    lidarD = min_can * 100;
    float max_ran = (max_element(ran.begin(),ran.end())-ran.begin() - ss/2) * lidarIncrement;
    //unit = cm   
    //std::cout << "ultra: " << ultra_msg[0]<< std::endl;


    // variables
    float frontSafetyDistance = 25;

    float calculatedAngle = 0.0;

    // drive
    if (lidarD > frontSafetyDistance)
    {
        //calculatedAngle = ultraDrive();

        calculatedAngle = max_ran * 30;
    }
    else
    {
        // output: calculatedAngle
        calculatedAngle = obstacleAvoidance();
        if (calculatedAngle < 0)
        {
            //turn right go back
            //ros::Time time_begin = ros::Time::now();
            drive_back(kXycarSteeringAangleLimit);
            sleep(1);
            drive(-kXycarSteeringAangleLimit);
            sleep(1);
        }
        else
        {
            drive_back(-kXycarSteeringAangleLimit);
            sleep(1);
            drive(kXycarSteeringAangleLimit);
            sleep(1);

        }
        //std::cout << "CACLULATED : " << calculatedAngle << std::endl;
    }
   
    float steeringAngle = std::max(static_cast<float>(-kXycarSteeringAangleLimit), std::min(static_cast<float>(calculatedAngle), static_cast<float>(kXycarSteeringAangleLimit)));

    speedControl(steeringAngle);
    drive(steeringAngle);
}


float MazeDrive::ultraDrive()
{
    // variable
    //ultra_array = getResult_ultra();
    int32_t ultraR = ultra_msg[5];
    int32_t ultraL = ultra_msg[7];
    float sideSafetyDistance = 20.0;
    float calculatedAngle;

    if (ultraR > sideSafetyDistance && ultraL > sideSafetyDistance)
    {
        //go straight
        calculatedAngle = 0.0;
        std::cout << "go straight" << std::endl;

        return calculatedAngle;

    }
    else
    {
        if (ultraR <= sideSafetyDistance)
        {
            // turn left
            // return calculatedAngle
            // (maybe hardcoding...)
            calculatedAngle = 20;
            std::cout << "right obs, turn left" << std::endl;
            return calculatedAngle;


        }
        if (ultraL <= sideSafetyDistance)
        {
            // turn right
            // return calculatedAngle
            // (maybe hardcoding...)
            calculatedAngle = -20;
            std::cout << "left obs, turn right" << std::endl;

            return calculatedAngle;

        }
    }
}


float MazeDrive::obstacleAvoidance()
{

    float distanceX = lidarD * cos(theta);
    float w = 20;       // car width (have to measure)
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
        return -calculatedAngle * 10;
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


void MazeDrive::drive_back(float steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(-mXycarSpeed);

    mPublisher.publish(motorMessage);
}

// output => lidarD, theta
void MazeDrive::lidarCallback(const sensor_msgs::LaserScan& message)
{
    lidarIncrement = message.angle_increment;
    int32_t lidarRangeLimit = ran.size();
    int32_t lidarXrange = 128;
    ran = message.ranges;
    // please check the function to get lidar data
    // i don't know how it works excectl


        
}


// output: ultraR, ultraL
void MazeDrive::ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& message_ultra)
{    
    // function to callback ultrasonic messages
    // ultra_msg.push_back(message_ultra.data[0]);
    //ultra_msg.push_back(message_ultra.data[1]);
    for (std::vector<int>::const_iterator it = message_ultra -> data.begin(); it != message_ultra -> data.end(); ++it)
    {
        ultra_msg.push_back(*it);
    }
}
} // namespace Maze
