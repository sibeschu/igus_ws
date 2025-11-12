# igus_ws - Einführung in die Robotik mit dem Igus-Rebel und ROS2

## Voraussetzungen 

- ROS2 Jazzy Jalisco
- Ubuntu 24.04. LTS

## Installation

```bash
git clone --recurse-submodules https://github.com/sibeschu/igus_ws.git
```

dann

```bash
cd igus_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

Die IP des Netzwerkadapters, der für die Verbindung mit dem Roboter genutzt wird muss sich im Netzwerk von 192.168.3.1 befinden.  
Diese wird festgelegt in `igus_rebel_ros2/src/igus_rebel/include/Rebel.hpp` (Zeile 88)

## Einstiegspunkte

Sourcen des Workspaces vom root-Ordner:  

`source install/setup.bash`

Starten der Simulation :  

```bash
ros2 launch igus_rebel_moveit_config igus_rebel_simulated.launch.py
```

```bash
src/
├── igus_student
│   ├── igus_student
│   │   ├── __init__.py
│   │   └── student_robot_control.py
```
In `student_robot_control.py` können Anweisungen für den Igus geschrieben werden.  
Mit `colcon build` im root-Ordner den Workspace erneut bauen, um Änderungen zu übernehmen.  
Ausführen mit `ros2 run igus_student student_robot_control`  

## Dokumentationen und Sourcecode

- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- [igus_rebel_ros2](https://bitbucket.org/truphysics/igus_rebel_ros2/src/main/)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/bafc21080c5c8e259dadbb309797949aee0dd950)
- [yolo_ros](https://github.com/mgonzs13/yolo_ros/tree/ecdc718c8600b0c0744e32477cd121de55be5e30)
