# 说明
1.armdriver为与单片机串口通讯的驱动功能包，主要用于获得传感器数据及发布控制舵机信息  
2.armteleop为键盘控制节点

# 再说明
1.此为轮趣六轴机械臂从ROS1移植到ROS2的项目  
2.armdriver中的msg不需要管  

ros2 run armdriver armdriver_node //机械驱动  
ros2 launch ros2 launch armmodel_moveit_config demo.launch.py
