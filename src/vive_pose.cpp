#include <iostream>
#include "vive_pose.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <openvr.h>
#include <unistd.h>
using namespace vr;

//You may want to put the global variables in a class, or just leave them here.
IVRSystem* vr_pointer = NULL;


static geometry_msgs::PoseStamped convertToGeometryMsg(HmdVector3_t controller_position)
{
    geometry_msgs::PoseStamped controller_msg;
    controller_msg.pose.position.x = controller_position.v[0];
    controller_msg.pose.position.y = controller_position.v[1];
    controller_msg.pose.position.z = controller_position.v[2];

    return controller_msg;
}

void ControllerPoseHandler::setMsg(geometry_msgs::PoseStamped controller1_msg, geometry_msgs::PoseStamped controller2_msg)
{
    this->controller1_msg = controller1_msg;
    this->controller2_msg = controller2_msg;
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

void Shutdown() 
{
	if (vr_pointer != NULL)
	{
		VR_Shutdown(); 
		vr_pointer = NULL;
	}
}

void GetControllerPositions(ControllerPoseHandler &poseHandler)
{   
    HmdVector3_t controller1_position;
    HmdVector3_t controller2_position;


    for (unsigned int deviceId=0;deviceId<k_unMaxTrackedDeviceCount;deviceId++) {

        TrackedDevicePose_t trackedDevicePose;
        VRControllerState_t controllerState;
        // HmdVector3_t position;

        ETrackedDeviceClass cl = 
            vr_pointer->GetTrackedDeviceClass(deviceId);

        if (!vr_pointer->IsTrackedDeviceConnected(deviceId))
            continue;

        if (cl == ETrackedDeviceClass::TrackedDeviceClass_Controller) {
            vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId, &controllerState, sizeof(controllerState), &trackedDevicePose);
            
            // trackedDevicePose
            ROS_INFO("Device id: %i", deviceId);
            // ROS_INFO("bool %i", trackedDevicePose.bPoseIsValid);
            if(deviceId == 1){
                poseHandler.statusController1 = trackedDevicePose.bPoseIsValid;
                if(trackedDevicePose.bPoseIsValid)
                    controller1_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
            }else if(deviceId == 2){
                poseHandler.statusController2 = trackedDevicePose.bPoseIsValid;
                if(trackedDevicePose.bPoseIsValid)
                    controller2_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
            }
            //perform actions with pose struct (see next section)
        }
    }

    geometry_msgs::PoseStamped controller1_msg = convertToGeometryMsg(controller1_position);
    geometry_msgs::PoseStamped controller2_msg = convertToGeometryMsg(controller2_position);

    double y = controller1_msg.pose.position.y;
    controller1_msg.pose.position.y = controller1_msg.pose.position.z;
    controller1_msg.pose.position.z = y;

    y = controller2_msg.pose.position.y;
    controller2_msg.pose.position.y = controller2_msg.pose.position.z;
    controller2_msg.pose.position.z = y;

    

    poseHandler.setMsg(controller1_msg, controller2_msg);

} 


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

    ros::Publisher controller1_pub = n.advertise<geometry_msgs::PoseStamped>("/vive/pose1", 1);
    ros::Publisher controller2_pub = n.advertise<geometry_msgs::PoseStamped>("/vive/pose2", 1);
    ControllerPoseHandler poseHandler;


    ROS_INFO("Started vive_pose node...");

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Read values position values from controllers & publish values
        GetControllerPositions(poseHandler);

        ROS_INFO("Controller 1 status: %i", poseHandler.statusController1);
        ROS_INFO("Controller 2 status: %i", poseHandler.statusController2);
        
        if(poseHandler.statusController1)
            controller1_pub.publish(poseHandler.controller1_msg);

        if(poseHandler.statusController2)
            controller2_pub.publish(poseHandler.controller2_msg);


        // Update sequence
        ros::spinOnce();
        loop_rate.sleep(); // Don't forget this! *
    }

    return 0;
}

