# Command line tips

## Version 1.0

- **Launch skin:**
```
$ roscore
$ roslaunch tum_skin_tests_fiad tum_skin_driver_fiad_tsu.launch
```
After successful launch: ``clear offsets; store offsets``
```
$ roslaunch tum_skin_tests_fiad pub_left_arm.launch
```


- **Run NAO:** (check the NAO's ip on 10.0.29.1(wireless router))
```
$ roscore
$ roslaunch nao_bringup nao_full_py.launch nao_ip:=10.0.29.2 roscore_ip:=127.0.0.1
$ roslaunch nao_bringup nao_full_py.launch nao_ip:=169.254.81.21 roscore_ip:=127.0.0.1
$ rosservice call /body_stiffness/enable "{}"
```

- **Simulation rviz:**
```
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
$ rosrun rviz rviz
```


- **Transfer package to my laptop:**
	1. Cannot locate node in xxxx package, please add **CMAKE_PREFIX_PATH**:
	There are some libraries in ***'/opt/ros/indigo/'***, such as 'lib', 'share' and 'include', 'tumskin', 'tumtools' 
	2. copy against permission: ``$ sudo cp -r home/ming/Desktop/tum_skin_driver_fiad/ opt/ros/indigo/include/ ``
	3. access against permission: ``$ sudo chmod -R 777 tum_skin_msgs_fiad/``
	4. `` $ rospack find tum_skin_msgs_fiad``
	5. python cannot import naoqi:

```
$ export PYTHONPATH=/home/ming/naoqi/naoqi-sdk-2.1.4.13-linux64/lib:${PYTHONPATH}
$ echo 'export PYTHONPATH=~/naoqi/pynaoqi-python2.7-2.1.2.17-linux64:$PYTHONPATH' >> ~/.bashrc
```


- **check IP port**
```
$ nmap --iflist
$ sudo netdiscover -r 192.168.0.0/24 -i eth1
```

## Version 2.0
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


