# Skin

http://wiki.ros.org/nao

这个是nao的一些基本的package，可以看一下；

http://wiki.ros.org/nao/Tutorials/Installation

这个是需要安装的一些package，dell电脑上应该已经都安装过了；

http://doc.aldebaran.com/2-1/naoqi/motion/control-walk-api.html

这个网页上有很多实用的控制nao行动的C++和Python指令，可以查看；

http://doc.aldebaran.com/2-1/index.html

nao相关的documentation，可以参考；

http://wiki.ros.org/nao/Tutorials/Getting-Started

在RViz中对nao仿真的相关知识，后面的内容过期了，可以看一下前面的内容，在RViz中显示nao有其他方法；

如果要学习ros可以看这个视频：https://www.youtube.com/watch?v=0BxVPCInS3M

ETH的ros课程，还挺有用的

https://www.youtube.com/playlist?list=PLAE85DE8440AA6B83 

这个是C++的视频


* launch skin:
	$ roscore
	$ roslaunch tum_skin_tests_fiad tum_skin_driver_fiad_tsu.launch
		after successful launch: clear offsets; store offsets  
	$ roslaunch tum_skin_tests_fiad pub_left_arm.launch

* run NAO (* check the nao's ip on 10.0.29.1(wireless router))
	$ roscore
	$ roslaunch nao_bringup nao_full_py.launch nao_ip:=10.0.29.2 roscore_ip:=127.0.0.1
	$ rosservice call /body_stiffness/enable "{}"


* simulation rviz
	$ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	$ rosrun rviz rviz

* Transfer package to my laptop:
	1. Cannot locate node in xxxx package, please add CMAKE_PREFIX_PATH
		There are some libraries in '/opt/ros/indigo/', such as 'lib', 'share' and 'include'
		'tumskin', 'tumtools'
		copy against permission: $ sudo cp -r home/ming/Desktop/tum_skin_driver_fiad/ opt/ros/indigo/include/
		access against permission: $ sudo chmod -R 777 tum_skin_msgs_fiad/			
	2. $ rospack find tum_skin_msgs_fiad
	3. python cannot import naoqi:
		$ export PYTHONPATH=/home/ming/naoqi/naoqi-sdk-2.1.4.13-linux64/lib:${PYTHONPATH}
		$ echo 'export PYTHONPATH=~/naoqi/pynaoqi-python2.7-2.1.2.17-linux64:$PYTHONPATH' >> ~/.bashrc

* check IP port
	$ nmap --iflist
	$ sudo netdiscover -r 192.168.0.0/24 -i eth1

* Write a new pub_id node
	1. create a new '.cpp' under catkin_ws/src/tum_skin_fiad/tum_skin_tests_fiad/src/Applications
	2. modigy the 'CMakeLists.txt' on this directory
	2. create a new '.launch' under catkin_ws/src/tum_skin_fiad/tum_skin_tests_fiad/launch/publish_ids