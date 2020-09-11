#include <ros/ros.h>
#include "path_planner.h"
#include <iostream>
#include "node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plannerNode"); 
    ros::NodeHandle n;

    if (ros::ok())
    {
	PlannerNode planner(n, false, 5, 5, 3);

	ROS_INFO_STREAM("EXPLORATORY PLANNER: LAUNCHED");

	while (ros::ok())
	{
	    planner.spinThread();
	}
    }
    else
    {
	std::cout << "ROS not ok..." << std::endl;
    }
}
