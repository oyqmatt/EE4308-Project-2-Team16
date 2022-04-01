#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

double dist_euc3d(geometry_msgs::Point src, geometry_msgs::Point tgt) {
    double Dx = tgt.x - src.x;
    double Dy = tgt.y - src.y;
    double Dz = tgt.z - src.z;
    return sqrt(Dx*Dx + Dy*Dy + Dz*Dz); 
}
double dist_euc3d(double src_x, double src_y, double src_z, geometry_msgs::Point tgt){
    double Dx = tgt.x - src_x;
    double Dy = tgt.y - src_y;
    double Dz = tgt.z - src_z;
    return sqrt(Dx*Dx + Dy*Dy + Dz*Dz); 
}

bool verbose;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}
double turtle_x = NaN, turtle_y = NaN, turtle_a;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
    auto &q = msg->pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    turtle_a = atan2(siny_cosp, cosy_cosp);
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    // ROS_INFO("Waiting: %d",(std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) ;
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);
    double end_x, end_y, end_z, end_vel_x, end_vel_y, duration; 
    geometry_msgs::Point target; 
    std::vector<geometry_msgs::Point> trajectory;
    int t = 0;
    bool flag = true;  // Flag to recalculate trajectory due to update of end
    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        //// IMPLEMENT ////
        if (state == TAKEOFF)
        {
            // Take off to height defined in YAML 
            if (flag) {
                // Disable Rotate
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);

                end_x = initial_x;
                end_y = initial_y;
                end_z = height + initial_z;
            }
            // Check
            else if (abs(z-height) < close_enough){
                state = TURTLE;
                msg_rotate.data = true;
                pub_rotate.publish(msg_rotate);
                flag = true;
            }
        }
        else if (state == TURTLE)
        {
            if (dist_euc(Position(x,y),Position(turtle_x,turtle_y)) < close_enough){
                state = GOAL;
                flag = true;
            }
            end_x = turtle_x;
            end_y = turtle_y;
            end_z = height + initial_z;
            flag = true;
        }
        else if (state == START)
        {
            if (!nh.param("/turtle/run", false) && dist_euc(Position(x,y),Position(initial_x,initial_y)) < close_enough)
            { // when the turtle reaches the final goal
                state = LAND;
                flag = true;
            }
            if (flag) {
                end_x = initial_x;
                end_y = initial_y;
                end_z = height + initial_z;
            }
        }
        else if (state == GOAL)
        {
            if (dist_euc(Position(x,y),Position(goal_x,goal_y)) < close_enough){
                state = START;
                flag = true;
            }
            if (flag) {
                end_x = goal_x;
                end_y = goal_y;
                end_z = height + initial_z;
            }
        }
        else if (state == LAND)
        {
            if (flag) {
                // Disable Rotate
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);

                end_x = initial_x;
                end_y = initial_y;
                end_z = initial_z;
            } 
        }

        // if (verbose)
            ROS_INFO_STREAM(" HMAIN : " <<  to_string(state));
        // Recalculate target if flag is true
        if (flag) {
            ROS_INFO("HMAIN: Recalc hector trajectory");
            if (state == LAND || state == TAKEOFF) {
                duration = abs(end_z - z)/ average_speed;
            }
            else {
                duration = dist_euc(Position(x,y),Position(end_x,end_y)) / average_speed;   
            }
            trajectory.clear();
            // Straight line
            for (double i = 0; i < duration; i += look_ahead) {
                geometry_msgs::Point traj;
                traj.x = end_x - i / duration * (end_x - x);
                traj.y = end_y - i / duration * (end_y - y);
                traj.z = end_z - i / duration * (end_z - z);
                trajectory.emplace_back(traj);
                // ROS_INFO("HERERERERERERE: %f, %f,%f, %d", average_speed,end_z, duration,trajectory.size());
                // ros::Duration(5).sleep();
            }
            // geometry_msgs::Point traj;
            // traj.x = end_x;
            // traj.y = end_y;
            // traj.z = end_z;
            // trajectory.emplace_back(traj);
            flag = false;

            // publish trajectroy to trajectory topic
            msg_traj.poses.clear();
            for (geometry_msgs::Point &pos : trajectory)
            {
                msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
                msg_traj.poses.back().pose.position.x = pos.x;
                msg_traj.poses.back().pose.position.y = pos.y;
                msg_traj.poses.back().pose.position.z = pos.z;
            }
            pub_traj.publish(msg_traj);

            // get new target
            t = trajectory.size() - 1; // last entry
            // pick the more distant target so turtlebot does not stop intermitently around very close targets when new path is generated
            // if (t > 15)
            //     t -= 15; // this is the average_speed * 15 * target_dt away
            target = trajectory[t];

            // publish first entry to target topic
            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = target.z;
            pub_target.publish(msg_target);
        }

        if (!trajectory.empty() && dist_euc3d(x,y,z,target) < close_enough)
        {
            if (--t < 0)
                t = 0; // in case the close enough for target triggers. indices cannot be less than 0.

            target = trajectory[t];
            // publish to target topic
            msg_target.point.x = target.x;
            msg_target.point.y = target.y;
            msg_target.point.z = target.z;
            pub_target.publish(msg_target);
        }
        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
