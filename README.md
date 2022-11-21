# vacuum_cleaner_robot

### Running

Source in every terminal, e.g.:
>source /root/Desktop/catkin_ws/devel/setup.bash

Run gazebo:
>roslaunch ima_turtlebot3 turtlebot3_house.launch

Run rviz:
>export TURTLEBOT3_MODEL=waffle_pi
>roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=MAP_PATH

This repo contains map files in folder map/

Run steering robot by keyboard for setting its home:
>rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Run smach viewer graph:
>rosrun smach_viewer smach_viewer.py

Run taking commands:
cd WORKSPACE_PATH/src/my_service/src
>python aut.py

or place automat files to /opt/ros/melodic/share/smach_viewer and run:
>rosrun smach_viewer aut.py

Run additional necessary nodes:
>rosrun my_service my_service_cleaning_home_node
>rosrun my_service my_service_go_home_node

Run detecting obstacles node (uses recognizing signs which is not located in this repo):
>python src/add_obstacles.py

<p>If you want the robot not only to see the obstacles but also to omit them <br>
while planning its way you need to modify your current planner to subscribe to <br>
this custom scanner topic.
</p>

### Result

- Taking command
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/cleaning_finished.png" width="900" height="450" />

- Going to base
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/go_to_base.png" width="900" height="450" />

- Undocking from base
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/undock_from_base.png" width="900" height="450" />

- Setting new base
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/set_base.png" width="900" height="450" />

- New base coordinates were uploaded
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/new_base_coordinates_uploaded.png" width="900" height="450" />

- Going to new base
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/go_to_new_base.png" width="900" height="450" />

- Undocking from new base
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/undock_from_new_base.png" width="900" height="450" />

- Cleaning the trash room
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/cleaning_trash_room.png" width="900" height="450" />

- Cleaning the office room
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/cleaning_office_room_1.png" width="900" height="450" />
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/cleaning_office_room_2.png" width="900" height="450" />

- Cleaning was finished
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/cleaning_finished.png" width="900" height="450" />

- Exit from program
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/exit.png" width="900" height="450" />

- How the robot sees the obstacles
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/obstacles_1.png" />
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/obstacles_2.png" />
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/obstacles_3.png" />
<img src="https://raw.githubusercontent.com/NastyaRush/vacuum_cleaner_robot/main/imgs/obstacles_4.png" />
