# Skin_NAO platform (Ubuntu 16.04 LTS 64 bit + ROS kinetic)

## How to use the gitlab file? (Files Navigation)
- This repository contains the whole source file that you need, clone it into your own ROS workspace, such as */catkin_ws/src*.
- After cloning use the command `catkin_make` to build up all the executable files, then you can run all the rosnodes.
- The folder */Previous_Work(Ubuntu14.04)* contains the work based on Ubuntu 14.04 before Sep. 2017., which relies on some previous libraries. So you cannot run them directly, if you would like to run this file, you need to contact [Florian Bergner](florian.bergner@tum.de)
- The folder */u16-skin-mohsen-20180125* contains all the skin libaries that we need for this 'NAO-Skin' platform, which are based on Ubuntu 16.04 LTS 64 bit and ROS: kinetic. If you change the computer you have to reinstall all the `.deb` packages inside. First the `.deb` packages in the folder */u16-skin-mohsen-20180125/tumtool*, then */tumskin* and finally */u16-skin-mohsen-20180125/kinetic*. Use the command `sudo dpkg -i *` in case that several packages are dependent on each other.
- The folder */tum_ics_skin_tutorials_fiad(backup)* contains the tutorials guide you to get start.

## How to connect and launch the skin?
### Hardware Preparation
1. Connect skin patch(es) to an interface box. Then connect the computer and the interface box through a switch.
2. Connect the switch with computer through a cable, set your labtop IP address as 192.168.0.3(actually the last number doesn's matter excpet 10, because the dafault IP address of skin patch is 192.168.0.10), netmask as 255.255.255.0, and gateway as 0.0.0.0.

### Tutorial
- You'd better to go into the folder */tum_ics_skin_tutorials_fiad(backup)*, it contains the original and uniform node.

### Launch the skin
1. With command `roslaunch tum_ics_skin_nao skin_driver_tsu.launch` you should be able to launch the skin patches, if not, try again. Actually, this a new launch file which is already modified. You can also launch the original one via `roslaunch tum_ics_skin_driver_events skin_driver_tsu.launch VERSION:=1`. Don't forget to give the argument! 
2. `roslaunch tum_ics_skin_nao load_and_view_patch.launch` you will see a window which lets you select the '.xml' file, after that you will see it publish the proximity sensor data.

### Create a 'xml' file for a new skin patch
1. `roslaunch tum_ics_skin_full_config full_config.launch`
2. You will see a GUI, click the capture twice, and save.
3. If you want to save for more patches, click the save in the second block instead of the first one.

