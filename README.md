# Farming Pollinator Robot

> God rewards ***the diligent*** . Reject ***FFFucking evil ways***.

$$
1 + 1 +1 > 3
$$

If you have any questions, please leave me **issues**.


## Team Member

* Gengming Liang
* Jianghui Bian
* Xiaoyu Guo

## Open Source composition

There are some folders and some files, as follow:

* **src/** —— Store ROS software packages
* **models/** —— Store the STEP source files of each 3D print
* **bash/** —— Store useful sh files
* **race_rule/** —— Store race rule
* **STM32_code/** —— Store the stm32 code of bottom plate
* **py_install/** —— Store rosmaster python library
* **py_files**/ —— Store important python files
* **yolo_model/** —— Store yolo BIN file
* **hardware_BOM.md** —— Bill of materials

## Important Items

1. Don't make car body get any knock to prevent imparting IMU accuracy.
2. When not starting IMU reading code, must remove IMU usb reading port.
3. When X3 RDK just turned on, must record first to active voice plate.

```
sust_record
```

```
sust_preplay
```

## Operation Procedure

1. Before race starts, do a test whether the voice plate play audio.

```
sust_record
```

```
sust_preplay
```

2. start ***upupup.launch.py*** that includes chassis nodes / ekf node / IMU node.

```
ros2 launch yahboomcar_bringup upupup.launch.py
# or
upupup
```

3. start ***debug_arm.launch.py*** that includes vision Yolo node / voice node / servo node.

```
ros2 launch yahboomcar_bringup debug_arm.launch.py
# or
debug_arm
```

4. start ***stp32l.launch.py*** that includes laser node.

```
ros2 launch ldlidar stp32l.launch.py
# or
lidar
```

7. After making sure that all nodes run normally, start race logic code.

```
cd ~/farming_ws/src/motion_controller/motion_controller/ && python3 game_process.py
# or
game
```

## Important child nodes

* Laser **stp23l**

```
ros2 launch ldlidar stp23l.launch.py
```

* Vision **Yolo**

```
ros2 launch dnn_node_example dnn_node_example.launch.py
```

* **IMU** data read

```
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

* servo board

```
ros2 run servo_control servo
```

* voice

```
ros2 run voice_broadcast voice
```

* Keyboard Control

```
ros2 run yahboomcar_ctrl yahboom_keyboard
```

## Common Terminal Commands

* scan **WIFI** networks

```
sudo nmcli device wifi rescan
```

* list **WIFI**

```
sudo nmcli device wifi list
```

* connect appointed **WIFI**

```
sudo wifi_connect "" ""
```

* Workspace Compilation

```
colcon build --packages-select <package_name>
```

* View hardware parameters of the audio board:
  1. The suffix P is play parameters.
  2. The suffix C is collect parameters.

```
ls /dev/sn*
```

* Publish topic by command line.

```
ros2 topic pub /voice_commands std_msgs/msg/Int32MultiArray '{data:[2,2,1,1]}' --once
```

* Set static IP.

```
nmcli connection modify 134 ipv4.method manual ipv4.addresses 192.168.3.33/24 ipv4.gateway 192.168.3.1 ipv4.dns 8.8.8.8
```

* Look at the gateway address. In the output of these commands, look for the default line, where the ***via*** field is followed by your gateway address.

```
ip route show
```

## Convenient Commands

* Do a test whether the voice board plays audios.

```
sust_record
```

* Pre-play the audio before the race.

```
sust_preplay
```

* Start upupup.launch.py

```
upupup
```

* Start race logic code

```
game
```

* View **readme** file

```
sust_books
```

* Open YoloV5 vision

```
vision
```

* List wifi

```
wifi_list
```

* Keyboard control

```
key
```
