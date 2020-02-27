#ifndef VIVE_POSE_H
#define VIVE_POSE_H

// #include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
#include <openvr.h>
// #include <unistd.h>

enum CONTROLLER_INDEX {VRC_LEFT, VRC_RIGHT};

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);

struct VR_ControllerButtons
{
    float trigger = 0;
};

// struct ControllerStatus
// {
//     bool status = 0;
// };

struct VR_Pose
{
    geometry_msgs::PoseStamped msg;
};

struct VR_Controller
{
    bool status = 0;
    VR_Pose pose;
    VR_ControllerButtons buttons;
};

class VR_ControlHandler {
    public:
        VR_Controller left;
        VR_Controller right;
        VR_Controller *pController[2];
        VR_ControlHandler(){
            this->pController[VRC_LEFT] = &this->left;
            this->pController[VRC_RIGHT] = &this->right;
        };
        char name[2][6] = { "Left", "Right"}; 
        void DebugPrint();
};


#endif //VIVE_POSE_H