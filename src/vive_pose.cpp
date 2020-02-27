#include <iostream>
#include "vive_pose.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <openvr.h>
#include <unistd.h>
using namespace vr;

//You may want to put the global variables in a class, or just leave them here.
IVRSystem* vr_pointer = NULL;


geometry_msgs::PoseStamped convertToGeometryMsg(HmdVector3_t controller_position, HmdQuaternion_t controller_orientation)
{
    geometry_msgs::PoseStamped controller_msg;
    controller_msg.header.frame_id = "panda_link0";
    controller_msg.pose.position.x = controller_position.v[0];
    controller_msg.pose.position.z = controller_position.v[1];
    controller_msg.pose.position.y = controller_position.v[2];

    controller_msg.pose.orientation.w = controller_orientation.w; //roll
    controller_msg.pose.orientation.x = controller_orientation.x;
    controller_msg.pose.orientation.y = controller_orientation.y;
    controller_msg.pose.orientation.z = controller_orientation.z;

    ROS_INFO("Orientation %f", controller_orientation.w);

    

    return controller_msg;
}

void ControllerPoseHandler::setMsg(geometry_msgs::PoseStamped controllerLeft_msg, geometry_msgs::PoseStamped controllerRight_msg)
{
    this->controllerLeft_msg = controllerLeft_msg;
    this->controllerRight_msg = controllerRight_msg;
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

void GetControllerPositions(ControllerPoseHandler &poseHandler)
{   
    HmdVector3_t controllerLeft_position;
    HmdVector3_t controllerRight_position;

    HmdQuaternion_t controllerLeft_orientation;
    HmdQuaternion_t controllerRight_orientation;

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
            
            if(role == TrackedControllerRole_LeftHand){
                poseHandler.controllerStatus.Left = trackedDevicePose.bPoseIsValid;
                if(trackedDevicePose.bPoseIsValid)
                    controllerLeft_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
                    controllerLeft_orientation = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
                    ROS_INFO("Left Trigger: %i", controllerState.rAxis[k_eControllerAxis_Trigger].x);

            }else if(role == TrackedControllerRole_RightHand){
                poseHandler.controllerStatus.Right = trackedDevicePose.bPoseIsValid;
                if(trackedDevicePose.bPoseIsValid)
                    controllerRight_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
                    controllerRight_orientation = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
                    ROS_INFO("Right Trigger: %i", controllerState.rAxis[k_eControllerAxis_Trigger].x);

                    
            }
            //perform actions with pose struct (see next section)
        }  
    }

    geometry_msgs::PoseStamped controllerLeft_msg = convertToGeometryMsg(controllerLeft_position, controllerLeft_orientation);
    geometry_msgs::PoseStamped controllerRight_msg = convertToGeometryMsg(controllerRight_position, controllerRight_orientation);

    poseHandler.setMsg(controllerLeft_msg, controllerRight_msg);
} 

// void SetControllerButtons(ControllerButtons &controllerButtons, VREvent_t event)
// {
//     switch( event.data.controller.button ) {
//         case k_EButton_Grip:
//             switch(event.eventType)	{
//                 case VREvent_ButtonPress:			
//                 break;

//                 case VREvent_ButtonUnpress:			
//                 break;
//             }
//         break;

//         case k_EButton_SteamVR_Trigger:  
//             switch(event.eventType) {
//                 case VREvent_ButtonPress:
//                     // ROS_INFO("BUTTON WAS PRESSED!");	
//                     controllerButtons.trigger = 1;	
//                 break;

//                 case VREvent_ButtonUnpress:	
//                     // ROS_INFO("BUTTON WAS UNPRESSED!");	
//                     controllerButtons.trigger = 0;		
//                 break;
//             }
//         break;

//         case k_EButton_SteamVR_Touchpad:
//             switch(event.eventType)	{
//                 case VREvent_ButtonPress:			
//                 break;

//                 case VREvent_ButtonUnpress:			
//                 break;

//                 case VREvent_ButtonTouch:			
//                 break;

//                 case VREvent_ButtonUntouch:			
//                 break;
//             }
//         break;

//         case k_EButton_ApplicationMenu:  
//             switch(event.eventType)	{
//                 case VREvent_ButtonPress: 			
//                 break;

//                 case VREvent_ButtonUnpress:			
//                 break;
//             }
//         break;
//     }
// }

// void GetControllerButtons(ControllerPoseHandler &poseHandler)
// {
//     VREvent_t event;
//     VRControllerState_t controllerState;
//     if(vr_pointer->PollNextEvent(&event, sizeof(event)))
//     {
//         ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(event.trackedDeviceIndex);
//         if(trackedDeviceClass != ETrackedDeviceClass::TrackedDeviceClass_Controller) {
//             return; //this is a placeholder, but there isn't a controller 
//             //involved so the rest of the snippet should be skipped
//         }
//         ETrackedControllerRole role = 
//         vr_pointer->GetControllerRoleForTrackedDeviceIndex(event.trackedDeviceIndex);
//         // controllerState = vr_pointer->GetControllerState(event.trackedDeviceIndex);
//         if (role == TrackedControllerRole_Invalid){
//             // The controller is probably not visible to a base station.
//             //    Invalid role comes up more often than you might think.
//         }
//         else if (role == TrackedControllerRole_LeftHand)
//         {
//             // Left hand
//             SetControllerButtons(poseHandler.controller1_buttons, event);
//         }
//         else if (role == TrackedControllerRole_RightHand)
//         {
//             // Right hand
//             SetControllerButtons(poseHandler.controller2_buttons, event);
//         }
//     }
// }


int main(int argc, char *argv[]) 
{

    std::cout << "Init variables...\n";


    EVRInitError eError = VRInitError_None;
    vr_pointer = VR_Init(&eError, VRApplication_Background);
    if (eError != VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n", 
            VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }

    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle n;

    ros::Publisher controller1_pub = n.advertise<geometry_msgs::PoseStamped>("/vive/controller/left/pose", 1);
    ros::Publisher controller2_pub = n.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/pose", 1);
    ros::Publisher controller1_pub_button = n.advertise<std_msgs::Float32>("/vive/controller/left/buttons", 1);
    ros::Publisher controller2_pub_button = n.advertise<std_msgs::Float32>("/vive/controller/right/buttons", 1);
    ControllerPoseHandler poseHandler;


    ROS_INFO("Started vive_pose node...");

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Read values position values from controllers & publish values
        GetControllerPositions(poseHandler);
        // GetControllerButtons(poseHandler);

        ROS_INFO("Controller/left/status: %i", poseHandler.controllerStatus.Left);
        ROS_INFO("Controller/right/status: %i", poseHandler.controllerStatus.Right);
        
        if(poseHandler.controllerStatus.Left)
            controller1_pub.publish(poseHandler.controllerLeft_msg);
            controller1_pub_button.publish(poseHandler.controllerLeft_buttons.trigger);

        if(poseHandler.controllerStatus.Right)
            controller2_pub.publish(poseHandler.controllerRight_msg);


        // Update sequence
        ros::spinOnce();
        loop_rate.sleep(); // Don't forget this! *
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