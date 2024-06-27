# 下地干活
## 1、重要事项
1. 车体不得一丝磕碰，预防影响 IMU 精度
2. 不启动 IMU 读取程序时，需拔掉 IMU 的 USB 读取接口
3. 不在运行程序的 X3 派和 IMU 需用小风扇对着吹
4. X3 派初开机时，先录音，激活音频板
```
cd ~/farming_ws/src/motion_controller/motion_controller/voice
```
```
sudo tinyplay ./voice/up.wav -D 0 -d 0
```

## 2、操作流程
1. 准备好三个终端，一个用于启动 launch，一个用于启动 python 文件，一个用于测试音频板
2. 比赛前测试音频板是否可以播放音频
```
sudo tinyplay ./voice/up.wav -D 0 -d 0
```
3. 用各种工具不断调整车体的初始位置，这个期间电脑掌控者不断测试音频版是否能播放音频
4. 在确保车体的初始位置正确和音频板能播放音频后，准备启动程序
5. 启动 upupup.launch.py
```
ros2 launch yahboomcar_bringup upupup.launch.py
```
6. 在确保 launch 正常运行后，运行 farming_visioner.py
```
python3 game_process.py
```

## 3、upupup.launch.py 中使用到的重要的子 launch
1. 激光
```
ros2 launch ldlidar stp23l.launch.py
```
2. 视觉 Yolo
```
ros2 launch dnn_node_example dnn_node_example.launch.py
```
3. IMU 数据读取
```
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```
4. 舵机板通讯
```
ros2 run yahboomcar_bringup servo
```

## Jun 29 调试流程
1. 更换 dnn_node_exmaple 节点读取模型的地址
2. 测试新训练模型的精度
3. PID 参数优化
4. 测试比赛逻辑代码
5. 规范场地标准

