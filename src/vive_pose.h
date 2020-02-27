#ifndef VIVE_POSE_H
#define VIVE_POSE_H

// #include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
#include <openvr.h>
// #include <unistd.h>

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);

struct ControllerButtons
{
    double trigger;
};

struct ControllerStatus
{
    bool Left = 0;
    bool Right = 0;
};

class ControllerPoseHandler {
    public:
        geometry_msgs::PoseStamped controllerLeft_msg; //Left
        geometry_msgs::PoseStamped controllerRight_msg; //Right
        void setMsg(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);
        // bool statusControllerLeft = 0;
        // bool statusControllerRight = 0;
        ControllerStatus controllerStatus;
        ControllerButtons controllerLeft_buttons;
        ControllerButtons controllerRight_buttons;
        // static geometry_msgs::PoseStamped convertToGeometryMsg(vr::HmdVector3_t controller_position);
        // static vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);


};

#endif //VIVE_POSE_H