#include "path_planner.h"
#include "node.h"
#include <iostream>
#include <vector>
#include <assert.h>
#include <limits>
#include <numeric>

PlannerNode::PlannerNode(ros::NodeHandle n, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : nh(n), const_velocity(const_velocity), v_max(v_max), v_const(v_const), max_f_gain(max_f_gain)
{
    times.reserve(std::numeric_limits<uint16_t>::max());
    rtimes.reserve(std::numeric_limits<uint16_t>::max());
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
        pub_health = nh.advertise<mur_common::diagnostic_msg>(HEALTH_TOPIC, 1);
        pub_lcones = nh.advertise<mur_common::cone_msg>(SORTED_CONES_LEFT_TOPIC, 1);
        pub_rcones = nh.advertise<mur_common::cone_msg>(SORTED_CONES_RIGHT_TOPIC, 1);
    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
        return 0;
    }

    ROS_INFO_STREAM("Planner: Path, visualisation, diagnostic publishers connected");
    return 1;
}

void PlannerNode::printVectors()
{
    for (int i = 0; i < X.size(); i++)
    {
        ROS_INFO_STREAM(X[i] << ' ' << Y[i] << ' ' << V[i]);
    }
}

void PlannerNode::spinThread()
{
    auto rstart = Clock::now();
    clearTempVectors();
    waitForMsgs();
    auto start = Clock::now();
    planner->update(cones, car_x, car_y, 
                    X, Y, V, 
                    cone_lx, cone_ly, cone_lcolour, 
                    cone_rx, cone_ry, cone_rcolour);
    auto end = Clock::now();
    pushPath();
    pushPathViz();
    pushSortedCones();
    cone_msg_received = false;
    odom_msg_received = false;
    auto rend = Clock::now();
    pushHealth(start, end, rstart, rend);
}

void PlannerNode::pushHealth(ClockTP& s, ClockTP& e, ClockTP& rs, ClockTP& re)
{
    mur_common::diagnostic_msg h;
    times.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(e - s).count());
    rtimes.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(re - rs).count());
    float mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    float rmean = std::accumulate(rtimes.begin(), rtimes.end(), 0.0) / rtimes.size();
    h.compute_time = std::chrono::duration_cast<std::chrono::microseconds>(e - s).count();
    h.compute_times = times;
    h.full_compute_times = rtimes;
    h.avg_times = mean;
    h.full_avg_times = rmean;

    h.header.stamp = ros::Time::now();

    pub_health.publish(h);
}

void PlannerNode::pushSortedCones() const
{
    mur_common::cone_msg cl;
    mur_common::cone_msg cr;

    cl.x.assign(cone_lx.begin(), cone_lx.end() - 1);
    cl.y.assign(cone_ly.begin(), cone_ly.end() - 1);

    cr.x.assign(cone_rx.begin(), cone_rx.end() - 1);
    cr.y.assign(cone_ry.begin(), cone_ry.end() - 1);

    pub_lcones.publish(cl);
    pub_rcones.publish(cr);
}

void PlannerNode::clearTempVectors()
{
    X.clear();
    Y.clear();
    V.clear();
    cones.clear();
    cone_lx.clear();
    cone_ly.clear();
    cone_lcolour.clear();
    cone_rx.clear();
    cone_ry.clear();
    cone_rcolour.clear();
}

void PlannerNode::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(X.size());

    for (int p = 0; p < X.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = X[p];
        item.pose.position.y = Y[p];
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}

void PlannerNode::pushPath()
{
   // publish mur_common::cone_msg
    mur_common::path_msg msg;
    msg.header.frame_id = "map";

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
	    ROS_ERROR_STREAM("'na' cone colour passed, skipping");
	}
	else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
	{
	    cones.push_back(Cone(msg.x[i], msg.y[i], 'r')); 
	}
    }
    cone_msg_received = true;
}


