#include "maze_runner/MazeDrive.hpp"

static constexpr double kFrameRate = 33.0;               ///< Frame rate

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc,argv,"Maze Runner");
    Maze::MazeDrive MD;
    ros::Rate rate(kFrameRate);

    while(ros::ok())
    {
        ros::spinOnce();

        MD.run();

    }

    return 0;
}