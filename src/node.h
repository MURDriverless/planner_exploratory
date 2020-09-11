#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "mur_common/cone_msg.h"
#include "mur_common/path_msg.h"
#include "cone.h"
#include <string>
#include <vector>
#include <memory>

#define ODOM_TOPIC "/mur/slam/Odom"
#define CONE_TOPIC "/mur/slam/cones"
#define PATH_TOPIC "/mur/planner/path"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"

class PlannerNode
{
public:
    PlannerNode(ros::NodeHandle, bool, float, float, float);
    void spinThread();

private:
    bool const_velocity;
    float v_max;
    float v_const;
    float max_f_gain;

    int launchSubscribers();
    int launchPublishers();
    void clearTempVectors();
    void pushPathViz();
    void pushPath();
    void waitForMsgs();
    void odomCallback(const nav_msgs::Odometry&);
    void coneCallback(const mur_common::cone_msg&);
    void callback(const nav_msgs::Odometry&, const mur_common::cone_msg&);
    void syncMsgs();
    bool cone_msg_received = false;
    bool odom_msg_received = false;

    std::unique_ptr<PathPlanner> planner;

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> V;
    
    std::vector<Cone> cones;
    float car_x;
    float car_y;

    ros::NodeHandle nh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_cones;
    
    ros::Publisher pub_path;
    ros::Publisher pub_path_viz;

    ros::Time now;
};

#endif // SRC_NODE_H
