# Documents structure:
/Nao_Manipulation: contains the ros source code for real NAO robot
/Vrep_Simulation: simualation on V-REP


# Real NAO manipulation:
## Preparation
- If you need to use the skin, you have to install some libraries of the skin at first.
Under the folder */u16_skin_driver* are all the skin libaries that we need for this 'NAO-Skin' platform, which are based on Ubuntu 16.04 LTS 64 bit and ROS: kinetic. If you change the computer you have to reinstall all the `.deb` packages inside. First the `.deb` packages in the folder */u16-skin-mohsen-20180125/tumtool*, then */tumskin* and finally */u16-skin-mohsen-20180125/kinetic*. Use the command `sudo dpkg -i *` in case that several packages are dependent on each other.

## Start and launch the skin
### Launch the skin
1. With command  `roslaunch tum_ics_skin_driver_events skin_driver_tsu.launch VERSION:=1`. Don't forget to give the argument! or you can modify the launch file in your own package.
2. `roslaunch tum_ics_skin_nao load_and_view_patch.launch` you will see a window which lets you select the '.xml' file, after that you will see it publish the proximity sensor data.

### Create a 'xml' file for a new skin patch
1. `roslaunch tum_ics_skin_full_config full_config.launch`
2. You will see a GUI, click the capture twice, and save.
3. If you want to save for more patches, click the save in the second block instead of the first one.

### Publish the sensor feedback
Roslaunch the launch file: *load_and_view_patch.launch* and you will get the sensor feedback.

### Train NAO robot with DMP
Rosrun the python file nao_train

# Vrep Simulation:
## Preparation:
### V-REP
### Choregraphe

## Train
- You should open the vrep scenario file *NAO.ttt* in V-REP at first.
- *nao_train.py* is for the training
- */trajectory_simulation* is for the trajectory visualization with forward kinematic.

