# vanttec_learning
ROS2 Tasks Solutions Package

Video: "Una disculpa chicos, después de guardar la sesión en mi compu me di cuenta que no tiene audio, entonces no le encontré sentido de subirlo jajj. Peeeero, todos los comandos que corrí en las demos vienen en la presentación. De nuevo, una disculpa, pero si les surgen dudas con las tareas o con lo que sea me pueden contactar a mi correo institucional: A01552369@tec.mx, o venir al laboratorio de VantTec (A4-431). - Max"

Presentación:
https://www.canva.com/design/DAF_sqpfmBM/lra3OneJylzecdU_JVLSZA/edit?utm_content=DAF_sqpfmBM&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton

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
