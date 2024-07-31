# 下地干活团队

> **天道酬勤，拒绝 TM 的歪门邪道！**


## 一、团队成员

* **梁耿明**
* **卞江辉**
* **郭晓宇**

## 二、重要事项

1. 车体不得一丝磕碰，预防影响 **IMU** 精度
2. 不启动 **IMU** 读取程序时，需拔掉 **IMU** 的 **USB** 读取接口
3. 不在运行程序的 **X3** 派和 **IMU** 需用小风扇对着吹
4. **X3** 派初开机时，先录音，激活音频板

```
sust_record
```

```
sust_preplay
```

## 三、操作流程

1. 准备好三个终端，一个用于启动 **launch**，一个用于启动 **python** 文件，一个用于测试音频板
2. 比赛前测试音频板是否可以播放音频

```
sust_record
```

```
sust_preplay
```

4. 用各种工具不断调整车体的初始位置，这个期间电脑掌控者不断测试音频版是否能播放音频
5. 在确保车体的初始位置正确和音频板能播放音频后，准备启动程序
6. 启动 **upupup.launch.py**

```
ros2 launch yahboomcar_bringup upupup.launch.py
# or
upupup
```

6. 在确保 **launch** 正常运行后，运行比赛逻辑代码

```
python3 game_process.py
# or
game
```

## 四、upupup.launch.py 中使用到的重要的子节点

* 激光 **stp23l**

```
ros2 launch ldlidar stp23l.launch.py
```

* 激光 **stp23**

```
ros2 launch ldlidar stp23.launch.py
```

* 视觉 **Yolo**

```
ros2 launch dnn_node_example dnn_node_example.launch.py
```

* **IMU** 数据读取

```
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

* 舵机板通讯

```
ros2 run yahboomcar_bringup servo
```

## 五、常用终端命令

* 扫描 **WIFI** 网络

```
sudo nmcli device wifi rescan
```

* 列出找出的 **WIFI**

```
sudo nmcli device wifi list
```

* 连接指定 **WIFI**

```
sudo wifi_connect "" ""
```

* 工作空间编译

```
colcon build --packages-select <package_name>
```

## 六、快捷命令

* 测试音频板是否能播放音频

```
sust_record
```

* 比赛前预播放音频

```
sust_preplay
```

* 启动集成 **launch**

```
upupup
```

* 启动比赛逻辑代码

```
game
```

* 查看 **readme** 文档

```
sust_books
```
