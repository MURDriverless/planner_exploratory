#include "path_planner.h"
#include "node.h"
#include <iostream>
#include <vector>

PlannerNode::PlannerNode(ros::NodeHandle n, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : nh(n), const_velocity(const_velocity), v_max(v_max), v_const(v_const), max_f_gain(max_f_gain)
{
    while (ros::ok())
    {
	launchSubscribers();
	launchPublishers();
    }
    
    waitForMsgs();

    planner = std::unique_ptr<PathPlanner>(new PathPlanner(car_x, car_y, cones, const_velocity, v_max, v_const, max_f_gain));

    now = ros::Time::now();
    pushPath();
    pushPathViz();

    cone_msg_received = false;
    odom_msg_received = false;
}

void PlannerNode::waitForMsgs()
{
    while (!cone_msg_received || !odom_msg_received) 
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
    return 1;
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
    path_viz_msg.header.frame_id = "path_viz";

    std::vector<geometry_msgs::PoseStamped> pose_stamped;
    pose_stamped.reserve(X.size());

    for (int p = 0; p < pose_stamped.size(); p++)
    {
	pose_stamped[p].header.stamp = now;	
	pose_stamped[p].header.frame_id = "path_viz";
	pose_stamped[p].header.seq = p;

	pose_stamped[p].pose.position.x = X[p];
	pose_stamped[p].pose.position.y = Y[p];
	pose_stamped[p].pose.position.z = 0.1;
	// Quarternion left blank
    }

    path_viz_msg.poses = pose_stamped;
    pub_path_viz.publish(path_viz_msg);
}

void PlannerNode::pushPath()
{
   // publish mur_common::cone_msg
    mur_common::path_msg path_msg;
    path_msg.header.frame_id = "path";
    path_msg.header.stamp = now;

    path_msg.x = X;
    path_msg.y = Y;
    path_msg.v = V;

    pub_path.publish(path_msg);
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
	    ROS_INFO_STREAM("'na' cone colour passed, skipping");
	}
	else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
	{
	    cones.push_back(Cone(msg.x[i], msg.y[i], 'r')); 
	}
    }
    cone_msg_received = true;
}

