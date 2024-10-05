# vanttec_learning
ROS2 Tasks Solutions Package

Tareas: 
https://drive.google.com/drive/folders/1-oMWIjd8tubTx2ufNs_YkqB1ytp8sMfa?usp=sharing

## Tarea #1
```Shell
# First terminal
ros2 run turtlesim turtlesim_node

# Second terminal
ros2 run vanttec_learning turtle_tf2_publisher_node

# Third terminal
ros2 run turtlesim turtle_teleop_key

# Fourth terminal
rviz2
```

## Tarea #2
```Shell
# First terminal
ros2 launch vanttec_learning tarea2_launch.py

# Second terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Tarea #3
```Shell
# First terminal
ros2 launch vanttec_learning tarea3_launch.py

# Second terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Tarea #4
```Shell
# First terminal
ros2 launch vanttec_learning tarea4_launch.py

# Second terminal
ros2 run turtlesim turtle_teleop_key
```