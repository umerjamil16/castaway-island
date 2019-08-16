# Castaway on Island

### Project Report
Project report is available [here](https://drive.google.com/file/d/1I6MWCM9pIgbApFIQM-kZr9dC0GSJqDxc/view)
### Setting up the simulation environment

Move into ```~/catkin_ws/src``` directory and clone this repository in a folder called ```ca_main``` by running following command:

```git clone https://github.com/umerjamil16/castaway-island.git ca_main```
Now, run 
```roscore```
And source the ```.bash``` file.
```source ~/.bashrc```
```source ~/catkin_ws/devel/setup.bash```

To launch the 2D world in stage, run:
```roslaunch ca_main 2d_world.launch```
then, for broadcasting TFs:
```roslaunch ca_main tf_broadcaster.launch ```

In another terminal, run the following command so that we can see the distance of castaway from the island center.
```rosrun ca_main dist_finder.py```

To change the position of the castaway in the 2D world, run the following command in a terminal:
```rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel```

Now, move the castaway outside the circle of radius r_i (As per calculations. In this simulation environment, the island radius of 3m for which r_i becomes 0.645m). Set the heading of castaway in the direction towards which it will move when we run the main program file. 

Now run the main program file as follows:
```rosrun ca_main strategy.py```

### Other useful commands
To manually launch a ```.world``` file in Stage, move into world directory of this repo and run:
```rosrun stage_ros stageros island.world ```
For TFs visualization:
```rosrun rqt_tf_tree rqt_tf_tree```

### ROS Packages used

 - [Stage_ros](http://wiki.ros.org/stage_ros)
