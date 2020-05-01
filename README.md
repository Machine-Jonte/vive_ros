# NOTE
Please note, this is a modified version of the original package [vive_ros](https://github.com/robosavvy/vive_ros) by robosavvy. Most cred goes to robosavvy and please check out the original package as well. The added files are vive_pose.cpp/.h which main purpose is to read the absolute pose in space (as I didn't get it to work with the ros_vive). I've probably missed some part, as in the demo video it is clear tracking of the controllers is possible. 

## Usage
(This was tested on Ubuntu 18.04 on a Dell XPS 9560).  
When everything is installed and set up (SteamVR, OpenVR and what not to get HTC-Vive work on Ubuntu), start the server to read the data:  
```
roslaunch vive_ros server_vr.launch
```
To close the server simply run:  
```
rosrun vive_ros close_server.sh
```

To start tracking the controllers run (added part from original package):  
```
rosrun vive_ros vive_pose
```


# vive_ros (original README content below)

Video example: [https://youtu.be/1hiX0f6UAew](https://youtu.be/1hiX0f6UAew)

## Installation instructions

### Download and build Valve's OpenVR SDK (most recently tested version):

      cd ~
      mkdir libraries
      cd libraries
      git clone https://github.com/ValveSoftware/openvr.git -b v1.3.22
      cd openvr
      mkdir build
      cd build
      cmake -DCMAKE_BUILD_TYPE=Release ../
      make

### Allow hardware access
Then plug-in VIVE to your computer and make sure you can see the devices on `/dev/hidraw[1-6]`.

Copy the file `60-HTC-Vive-perms.rules` to the folder `/etc/udev/rules.d`. Then run:

      sudo udevadm control --reload-rules && sudo udevadm trigger

### Install Steam and SteamVR

Install Steam:
      
      sudo apt install steam

Run Steam:
      
      steam

Setup or log in into your Steam account and install SteamVR app from Steam store.

Steam files should be located in: `~/.steam/steam`

SteamVR files should be located in: `~/.steam/steam/steamapps/common/SteamVR`

### Configure display.

Go to your OS display options to enable HMD's display.

## Usage

Before start:

* Make sure VIVE is present as several `/dev/hidraw*` and you have access permissions.
* Make sure VIVE display is enabled as extended view.
* Libraries and Steam are present on the folders described by `INSTALL.md`.

Procedure:

1. Launch the SteamVR's `vrserver` by launching the file: `roslaunch vive_ros server_vr.launch`
2. Launch the node: `roslaunch vive_ros vive.launch`
3. To close the node you can `Ctrl+C`. To close the vr server you have to kill the process. For convenience: `rosrun vive_ros close_servervr.sh`
