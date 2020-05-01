// Written by Jonathan Österberg. This is the file that is added to the vive_ros package.
// If questions, contact me on jonte_ost@live.com

#include <iostream>
#include "vive_pose.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <openvr.h>
#include <unistd.h>
#include <tf2_ros/transform_broadcaster.h>
#include <bitset>



using namespace vr;

//You may want to put the global variables in a class, or just leave them here.
IVRSystem* vr_pointer = NULL;


geometry_msgs::Pose Remap(geometry_msgs::Pose pose)
{
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;

    pose.position.x = -z;
    pose.position.y = -x;
    pose.position.z = y;

    return pose;
}

void VR_ControlHandler::DebugPrint()
{
    for(int i = 0; i < 2; i++){
        printf("\n%-15s%-10s%-10s%-10s%-10s\n", this->name[i], "x", "y", "z", "w");
        printf("%-15s%-10.4f%-10.4f%-10.4f%-10.4f\n", "Pose", this->pController[i]->pose.msg.pose.position.x, this->pController[i]->pose.msg.pose.position.y, this->pController[i]->pose.msg.pose.position.z, 0.0);
        printf("%-15s%-10.4f%-10.4f%-10.4f%-10.4f\n", "Orientation", this->pController[i]->pose.msg.pose.orientation.x, this->pController[i]->pose.msg.pose.orientation.y, this->pController[i]->pose.msg.pose.orientation.z, this->pController[i]->pose.msg.pose.orientation.w);
        printf("%-15s%-10.4f\n", "Trigger", this->pController[i]->buttons.trigger);
        printf("%-15s%-10i\n", "Status", this->pController[i]->status);
    }
}


/* This function is very much hardcoded for the panda robot. If use elsewhere add char as input*/
geometry_msgs::PoseStamped MakeGeometryMsgFromData(HmdVector3_t controller_position, HmdQuaternion_t controller_orientation, const char* frame_id)
{
    geometry_msgs::PoseStamped controller_msg;
    controller_msg.header.frame_id = "base";
    // controller_msg.header.frame_id = "controller";
    // controller_msg.header.frame_id = frame_id;
    controller_msg.pose.position.x = controller_position.v[0];
    controller_msg.pose.position.y = controller_position.v[1];
    controller_msg.pose.position.z = controller_position.v[2];

    controller_msg.pose = Remap(controller_msg.pose);

    controller_msg.pose.orientation.w = controller_orientation.w; //roll
    controller_msg.pose.orientation.x = -controller_orientation.z;
    controller_msg.pose.orientation.y = -controller_orientation.x;
    controller_msg.pose.orientation.z = controller_orientation.y; 

    // controller_msg.pose.orientation.w = controller_orientation.w; //roll
    // controller_msg.pose.orientation.x = controller_orientation.x;
    // controller_msg.pose.orientation.y = controller_orientation.y;
    // controller_msg.pose.orientation.z = controller_orientation.z;

    return controller_msg;
}


HmdQuaternion_t GetRotation(HmdMatrix34_t matrix) 
{
	HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

HmdVector3_t GetPosition(HmdMatrix34_t matrix) 
{
	HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

void GetControllerState(VR_ControlHandler &controlHandler)
{   

    // HmdVector3_t controller_position;
    // HmdQuaternion_t controller_orientation;

    for (unsigned int deviceId=0;deviceId<k_unMaxTrackedDeviceCount;deviceId++) {

        TrackedDevicePose_t trackedDevicePose;
        VRControllerState_t controllerState;

        ETrackedDeviceClass cl = vr_pointer->GetTrackedDeviceClass(deviceId);

        if (!vr_pointer->IsTrackedDeviceConnected(deviceId))
            continue;

        if (cl == ETrackedDeviceClass::TrackedDeviceClass_Controller) {
            vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId, &controllerState, sizeof(controllerState), &trackedDevicePose);
            ETrackedControllerRole role = vr_pointer->GetControllerRoleForTrackedDeviceIndex(deviceId);
            vr_pointer->GetControllerState(deviceId, &controllerState, sizeof(controllerState));
            
            // Decide which of the controllers to update
            int controllerIndex;
            role == TrackedControllerRole_LeftHand ? controllerIndex = VRC_LEFT : controllerIndex = VRC_RIGHT;


            controlHandler.pController[controllerIndex]->buttons.trigger = controllerState.rAxis[1].x; // 1 is trigger
            controlHandler.pController[controllerIndex]->buttons.touchpad[0] = controllerState.rAxis[0].x; // touchpad x axis
            controlHandler.pController[controllerIndex]->buttons.touchpad[1] = controllerState.rAxis[0].y; // touchpad y axis
            // printf("1: %f, 2: %f, 3: %f, 4: %f, 5: %f\n", controllerState.rAxis[0].y, controllerState.rAxis[1].y, controllerState.rAxis[2].y, controllerState.rAxis[3].y, controllerState.rAxis[4].y);
            // printf("BitFlag: %u", controllerState.ulButtonPressed);
            std::bitset<8> digitalButtons(controllerState.ulButtonPressed);
            // std::cout << "controllerState = " << x[2] << std::endl;
            // printf("This is %i \n", (int) digitalButtons[1]);
            controlHandler.pController[controllerIndex]->buttons.menu = (int) digitalButtons[1];
            controlHandler.pController[controllerIndex]->buttons.grip = (int) digitalButtons[2];
            
            controlHandler.pController[controllerIndex]->status = trackedDevicePose.bPoseIsValid;

            if(trackedDevicePose.bPoseIsValid)
            {
                HmdVector3_t controller_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
                HmdQuaternion_t controller_orientation = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
                controlHandler.pController[controllerIndex]->pose.msg = MakeGeometryMsgFromData(controller_position, controller_orientation, "panda_link0");
            }

            //perform actions with pose struct (see next section)
        }  
    }

} 

int main(int argc, char *argv[]) 
{
    // Init vr stuff
    EVRInitError eError = VRInitError_None;
    vr_pointer = VR_Init(&eError, VRApplication_Background);
    if (eError != VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n", 
            VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }

    // init ROS
    ros::init(argc, argv, "vive_publisher");
    ros::NodeHandle n;


    ros::Publisher pub_controller_pose[] = {n.advertise<geometry_msgs::PoseStamped>("/vive/controller/left/pose", 1), n.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/pose", 1)};
    ros::Publisher pub_controller_trigger[] = {n.advertise<std_msgs::Float32>("/vive/controller/left/trigger", 1), n.advertise<std_msgs::Float32>("/vive/controller/right/trigger", 1)};
    ros::Publisher pub_controller_menu[] = {n.advertise<std_msgs::Int32>("/vive/controller/left/buttons/menu", 1), n.advertise<std_msgs::Int32>("/vive/controller/right/buttons/menu", 1)};
    ros::Publisher pub_controller_grip[] = {n.advertise<std_msgs::Int32>("/vive/controller/left/buttons/grip", 1), n.advertise<std_msgs::Int32>("/vive/controller/right/buttons/grip", 1)};
    ros::Publisher pub_controller_touchpad[] = {n.advertise<std_msgs::Float32MultiArray>("/vive/controller/left/touchpad", 1), n.advertise<std_msgs::Float32MultiArray>("/vive/controller/right/touchpad", 1)};

    // ros::Publisher pub_controllerRight_pose = n.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/pose", 1);
    // ros::Publisher pub_controllerLeft_button = n.advertise<std_msgs::Float32>("/vive/controller/left/trigger", 1);
    // ros::Publisher pub_controllerRight_button = n.advertise<std_msgs::Float32>("/vive/controller/right/trigger", 1);

    VR_ControlHandler controlHandler;
    


    ROS_INFO("Started vive_pose node...");

    ros::Rate loop_rate(10); // Freq.
    // Read values position values from controllers & publish values
    while (ros::ok()) {
        GetControllerState(controlHandler);
        // controlHandler.DebugPrint();

        for(int i = 0; i < 2; i++)
        {
            if(controlHandler.pController[i]->status)
            {
                std_msgs::Float32MultiArray touchpad_msg;
                touchpad_msg.data = controlHandler.pController[i]->buttons.touchpad;
                // touchpad_msg.layout.dim[0].size = controlHandler.pController[i]->buttons.touchpad.size();
                // touchpad_msg.layout.dim[0].stride = 1;
                // touchpad_msg.layout.dim[0].label = "x";
                
                pub_controller_pose[i].publish(controlHandler.pController[i]->pose.msg);
                pub_controller_trigger[i].publish(controlHandler.pController[i]->buttons.trigger);
                pub_controller_menu[i].publish(controlHandler.pController[i]->buttons.menu);
                pub_controller_grip[i].publish(controlHandler.pController[i]->buttons.grip);
                pub_controller_touchpad[i].publish(touchpad_msg);
            }
        }

        // Update sequence
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



void Shutdown() 
{
	if (vr_pointer != NULL)
	{
		VR_Shutdown(); 
		vr_pointer = NULL;
	}
}