#ifndef AUTONOMOUS_NAVIGATION_ROBOT_ROBOT_H
#define AUTONOMOUS_NAVIGATION_ROBOT_ROBOT_H

#include <vector>
#include <string>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>
#include <omnibot_msgs/pose_xyt_t.hpp>
#include <omnibot_msgs/robot_path_t.hpp>
#include <omnibot_msgs/omni_motor_command_t.hpp>

#include "../utils/math_helpers.h"
#include "../utils/graph_utils.h"

#define MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
#define MBOT_MOTOR_COMMAND_CHANNEL "MBOT_MOTOR_COMMAND"
#define SLAM_POSE_CHANNEL "SLAM_POSE"
#define CONTROLLER_PATH_CHANNEL "CONTROLLER_PATH"
#define BOTGUI_GOAL_CHANNEL "BOTGUI_GOAL"
#define TIMEOUT 5000

static lcm::LCM lcmInstance(MULTICAST_URL);
static bool pose_received_;
static omnibot_msgs::pose_xyt_t robotPose;
lcm::LCM::HandlerFunction<omnibot_msgs::pose_xyt_t> poseCallback;

static void drive(const float vx, const float vy, const float wz)
{
    omnibot_msgs::omni_motor_command_t cmd;
    cmd.utime = getTimeMicro();
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;

    lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
};

static bool drivePath(std::vector<Cell>& path, GridGraph& graph)
{
    if (path.size() <= 1) return false;

    omnibot_msgs::robot_path_t msg;
    int utime = getTimeMicro();
    for (Cell& cell : path)
    {
        omnibot_msgs::pose_xyt_t pose;
        pose.utime = utime;
        auto position = cellToPos(cell.i, cell.j, graph);
        pose.x = position[0];
        pose.y = position[1];
        pose.theta = 0;
        msg.path.push_back(pose);
    }

    msg.path_length = path.size();
    msg.utime = utime;

    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &msg);

    return true;
}

static void handle()
{
    if (lcmInstance.handleTimeout(TIMEOUT) == 0)
    {
        std::cout << "WARNING: No SLAM pose data received.\n";
        pose_received_ = false;
    }
}

static void initPoseListener()
{
    pose_received_ = false;
    poseCallback = [](const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                      const omnibot_msgs::pose_xyt_t* msg)
    {
        robotPose = *msg;
        pose_received_ = true;
    };
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, poseCallback);
    handle();
}

static void getPose(float& x, float& y, float& theta)
{
    x = robotPose.x;
    y = robotPose.y;
    theta = robotPose.theta;
};

static bool hasPose()
{
    return pose_received_;
};

static bool isReady(const double timeout = 0)
{
    fd_set rfds;
    int lcmCheck = lcmInstance.getFileno();

    FD_ZERO(&rfds);
    FD_SET(lcmCheck, &rfds);

    struct timeval tv;
    tv.tv_sec = static_cast<int>(timeout);
    tv.tv_usec = static_cast<int>((timeout - tv.tv_sec) * 1e6);

    int retval = select(lcmCheck + 1, &rfds, NULL, NULL, &tv);

    if (retval == -1)
    {
        return false;
    }
    else if (retval)
    {
        return true;
    }

    return false;
}

static bool handleOnce(const double timeout = 0)
{
    if (isReady(timeout))
    {
        return (lcmInstance.handle() == 0);
    }

    return true;
}

#endif  // AUTONOMOUS_NAVIGATION_ROBOT_ROBOT_H
