#include "maze_runner/maze_drive.hpp"


int32_t main(int32_t argc, char** argv)
{
    ros::init(argc,argv,"Maze Runner");

    while(ros::ok())
    {
        ros::spineOnce();
        Maze::MazeDrive MD;
        MD.run();
    }

    return 0;
}