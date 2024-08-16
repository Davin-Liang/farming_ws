echo "source ~/farming_ws/install/setup.bash" >> ~/.bashrc
echo "alias upupup='cd && ros2 launch yahboomcar_bringup upupup.launch.py'" >> ~/.bashrc
echo "alias debug_arm='cd && ros2 launch yahboomcar_bringup debug_arm.launch.py'" >> ~/.bashrc
echo "alias lidar='ros2 launch ldlidar stp32l.launch.py'" >> ~/.bashrc
echo "alias game='cd ~/farming_ws/src/motion_controller/motion_controller/ && python3 game_process.py'" >> ~/.bashrc
echo "alias sust_books='cat ~/farming_ws/README.md'" >> ~/.bashrc
echo "alias vision='cd && ros2 launch dnn_node_example dnn_node_example.launch.py'" >> ~/.bashrc
echo "alias key='ros2 run yahboomcar_ctrl yahboom_keyboard'" >> ~/.bashrc

source ~/.bashrc