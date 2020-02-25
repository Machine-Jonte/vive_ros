#ifndef VIVE_POSE_H
#define VIVE_POSE_H

// #include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
#include <openvr.h>
// #include <unistd.h>


class ControllerPoseHandler {
    public:
        geometry_msgs::PoseStamped controller1_msg;
        geometry_msgs::PoseStamped controller2_msg;
        void setMsg(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);
        bool statusController1;
        bool statusController2;
        // static geometry_msgs::PoseStamped convertToGeometryMsg(vr::HmdVector3_t controller_position);
        // static vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);


};

#endif //VIVE_POSE_H