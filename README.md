
# AMR - Sanitizer Robot

## Final Project 

**Students involved in the project:**
- Bertamè Sebastiano - sebastiano.bertame@studio.unibo.it
- Elio Alberto - alberto.elio@studio.unibo.it
- Villari Francesco - francesco.villari@studio.unibo.it


**Instructor:**
- Gianluca Palli - gianluca.palli@unibo.it
- Department: DEI - LAR
- University: University of Bologna
- Address: Viale del Risorgimento 2, 40136 Bologna



##  Task1 - Simulation setup
In order to setup the simulation we used the ROS2 Nav2 package. With the following command from the terminal:

```Bash
export TURTLEBOT3_MODEL=burger

GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
we exported the robot model 'burger', we initialized the Gazebo world and eventually we started Gazebo and RViz with the TurtleBot3 big_house world.
In order to use the big_house scenario we modified the configuration file of nav2 to add that specific world. The modified files are uploaded in the GitHub repository.

![bighouse](https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/a512c2ce-dc91-45dd-a65c-3df146bb8e05)


To launch the simulation of the mapping and/or the sanitization we implemented a system that allows to pass some arguments according to the tasks that the user wants to be completed. When the simulation is launched, the user is asked to provide some parameters, that are:
- the x and y coordinates of the initial pose of the robot
- the task to perform: enter 0 for the mapping or 1 for the sanitization
- the localization method to be used: 0 for imposing as initial estimation for the Adaptive Monte Carlo Localization method the pose given by the user or 1 to use the global localization service of this method

We decided to split the simulation launch in two different launch files. The first one is in charge to start the TurtleBot3 simulation inside the package nav2_bringup that starts Gazebo and RViz with the world set in the parameters files (in our case we changed it in big_house) and asks for the parameters metioned above. The parameters passed by the user on the terminal are then written in a text file and are read by the second launch file. This last, starts all the specitìfic nodes that have been created to perform all the tasks of the project. The nodes are described below in the next sections.

## Task 2 - Mapping of the scenario
Following the setup of our simulation, we embarked on the mapping task by incorporating the 'slam:=True' parameter into our command line, as shown below. This step is crucial for initiating the SLAM (Simultaneous Localization and Mapping) process:

```Bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True
```

In our quest for an effective mapping strategy, we opted for a navigation approach based on **frontier exploration**. Our choice was driven by the strength of this method, that are its capability to achieve an exhaustive environmental coverage and its proficiency in rapidly mapping areas. Frontier exploration stands out for its applicability in environments populated with static obstacles, attributed to its minimal computational requirements and the absence of necessity for intricate adjustments in consistent settings.

For the implementation we used [this repository](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) designed for nav2 compatibility, making it an ideal match for our objectives. Subsequent testing revealed the repository's code to be highly effective in fulfilling our mapping requirements, leading us to adopt this solution for the project's duration.
In the following image the concept of frontier is shown.


![Frontier](https://hackmd.io/_uploads/Hk6UjK9sp.png)

The code implmented in the repository subscribes to the *Global Occupancy Grid topic* to determine known areas of the map. Through nested iterations, it examines each point and its adjacent points to identify frontiers. Once frontiers are established, the algorithm selects the central point of the furthest frontier and designates a target location goal for the robot. Utilizing nav2, the robot then plots a path to the specified position. This process is iterative and concludes when no additional frontiers are detected, ensuring a thorough mapping of the environment.

The command to launch the exploraiton is the following:

```Bash
ros2 run nav2_wfd explore
```
The main problem we faced on during the implementation of this  method has been the lackness of frontiers even if the map was not fully complete.
In fact, due to the specific lidar model and mapping method of 'bruger' device, it is not allowed to mark an area as free space in no obstacles are seen. This means that in big empty spaces (like *turtlebot empty world*) 'burger' is not capable to achieve a good mapping result.
The image below shows the most frequent instance of this problem we ecountered during developing the mapping task:

![frontiere](https://hackmd.io/_uploads/ByMSs3oi6.png)

When escaping from a room, if the distance to the opposite wall is larger than the laser range there is no chance to mark as free space any point in the second room. This means that no frontiers can be generated.

The only way to solve this problem without making changes in the mapping algorithm is providing a lidar device with larger maximum range.

https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/0a0650db-33a7-4ca7-a109-a0bdf2bd49dc

## Task 3 - Localization in the world 
For the localization we have used the Adaptive Monte Carlo Localization imlemented in the Nav2 package. To localize the robot, this algorithm uses a particle filter, a localization method based on the continuous update of the probability of the robot to be positioned in a specific point of the map. The algorithm starts with the initialization of a high number of points/particles (e.g. 100) uniformly spawned on the map. Then, the following steps are iteratively performed:

- the weights of the particles are updated basing on how much the position on of the point in the map matches the sensor reading. Namely, if the obstacles near the point are the same as the ones seen by the LIDAR the weight of the point increases.
- the covariance matrix is updated. This matrix shows the variance between the estimations of the spacial and angular coordinates of the robot.
- a resample of the particles is executed in order to relocate the points close to the ones with the higher weights.

The position estimation is performed for the entire duration of the simulation.
As mentioned in the task 1 Section, to determine the initial position of the robot we implemented two different methods. With the first one, we impose the initial pose as the chosen by the user so the covariance will be immediately very low and the robot will soon start to move. With the second approach, we initialize the global localization service obtaining a cloud of particles randomly spread in the map. Then our node, called init_localization_node, makes the robot to start spinning for a given amount of time and then to move to a random waypoint set at a distance of 0.55 meters.


https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/eb879dd2-37ac-4dc4-86b8-64a156305b57

## Task 4 - Sanitization of the environment
To sanitize the envirnoment we discretized the map with a resolution of 0.2 meters and applied the following energy distribution law:
$$E(x, y, t) = \int_0^t \frac{P_l}{(x - p_x(\tau))^2 + (y - p_y(\tau))^2} d\tau$$
To implement it in the code we used the discrtized equation that is:
$$E(x, y, t) = \int_0^t \frac{P_l}{(x - p_x(\tau))^2 + (y - p_y(\tau))^2} d\tau$$

The sanitization process has been divided in three steps:
- UV power evaluation: for every point of the discretized map we computed the related instant power value using the node power_publisher_node. This latter, computes the power according to the distance from the robot and shape of the room, provided by the /laser_scan topic. With the message of the laser scan, we generate a polygon using as vertices the points in which the laser bumps into. The power is then evaluated only for the point of the map inside the polygon and it's set as 0 for the points outside the polygon.
- Energy evaluation: 



https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/7c7eac71-2844-4ace-b5ae-005f37467ff0







Suppose that:
- In order to kill the coronavirus, the robot is equipped with a set of UV lamps able to spread all around the robot a light power \(P_l = 100 µW/m^2\);
- The UV energy \(E\) at point \((x, y)\) and time \(t\) can be computed as:
  
  where \(p_x(t)\) and \(p_y(t)\) represent the robot position along the x and the y axis respectively at time \(t\);
- Any obstacle completely stops the UV power propagation;
- The light power emitted at a distance lower than 0.1 meters from the robot is zero due to the robot encumbrance;
- The room of interest can be discretized as a grid with resolution 0.2 m. 

By sampling the process with sample time $` \Delta t `$, the UV energy \(E\) at point \((x, y)\) and sample time \(k\) can be rewritten as:
  $$
  E(x, y, k) = \sum_{i=0}^{k} \frac{P_l \Delta t}{(x - p_x(i \Delta t))^2 + (y - p_y(i \Delta t))^2}
  $$




