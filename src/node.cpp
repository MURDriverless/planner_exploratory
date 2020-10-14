#include "path_planner.h"
#include "node.h"
#include <iostream>
#include <vector>
#include <assert.h>

PlannerNode::PlannerNode(ros::NodeHandle n, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : nh(n), const_velocity(const_velocity), v_max(v_max), v_const(v_const), max_f_gain(max_f_gain)
{
    launchSubscribers();
    launchPublishers();
    
    waitForMsgs();

    try
    {
	this->planner = std::unique_ptr<PathPlanner>(new PathPlanner(car_x, car_y, cones, const_velocity, v_max, v_const, max_f_gain));
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);	
    }

    ROS_INFO_STREAM("Planner: Planner initialized");

    now = ros::Time::now();

    cone_msg_received = false;
    odom_msg_received = false;
}

void PlannerNode::waitForMsgs()
{
    while (!cone_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
	ros::Duration(0.005).sleep();
    }
}

int PlannerNode::launchSubscribers()
{
    try
    {
	sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PlannerNode::odomCallback, this);
	sub_cones = nh.subscribe(CONE_TOPIC, 1, &PlannerNode::coneCallback, this);
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);
	return 0;
    }
    ROS_INFO_STREAM("Planner: Odometry and cone subscribers connect");
    return 1;
}

int PlannerNode::launchPublishers()
{
    try
    {
	pub_path = nh.advertise<mur_common::path_msg>(PATH_TOPIC, 1);
	pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);
	return 0;
    }
    ROS_INFO_STREAM("Planner: Path and visualizer publishers connected");
    return 1;
}

void PlannerNode::printVectors()
{
    for (int i = 0; i < X.size(); i++)
    {
	std::cout << X[i] << ' ' << Y[i] << ' ' << V[i] << std::endl;
    }
}

void PlannerNode::spinThread()
{
    clearTempVectors();
    waitForMsgs();
    planner->update(cones, car_x, car_y, X, Y, V);
    now = ros::Time::now();
    pushPath();
    pushPathViz();
    cone_msg_received = false;
    odom_msg_received = false;
}

void PlannerNode::clearTempVectors()
{
    X.clear();
    Y.clear();
    V.clear();
    cones.clear();
}

void PlannerNode::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.stamp = now;
    path_viz_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(X.size());

    for (int p = 0; p < X.size(); p++)
    {
	geometry_msgs::PoseStamped item; 
	item.header.frame_id = "map";
	item.header.seq = p;
	item.header.stamp = now;
	item.pose.position.x = X[p];
	item.pose.position.y = Y[p];
	item.pose.position.z = 0.0;

	poses.push_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}

void PlannerNode::pushPath()
{
   // publish mur_common::cone_msg
    mur_common::path_msg msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now;

    msg.x = X;
    msg.y = Y;
    msg.v = V;

    pub_path.publish(msg);
}

void PlannerNode::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    odom_msg_received = true;
}

void PlannerNode::coneCallback(const mur_common::cone_msg &msg)
{
    for (int i = 0; i < msg.x.size(); i++)
    {
	if (msg.colour[i] == "BLUE")
	{
	    cones.push_back(Cone(msg.x[i], msg.y[i], 'b')); 
	}
	else if (msg.colour[i] == "YELLOW")
	{
	    cones.push_back(Cone(msg.x[i], msg.y[i], 'y')); 
	}
	else if (msg.colour[i] == "na")
	{
	    std::cout << "'na' cone colour passed, skipping" << std::endl;
	}
	else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
	{
	    cones.push_back(Cone(msg.x[i], msg.y[i], 'r')); 
	}
    }
    cone_msg_received = true;
}


