# Gazebo-Map-Actor-Plugin
A gazebo actor plugin that utilizes the map of the environment and graph search methods to generate random actor trajectories that don't pass through walls, furniture, etc.

The actors start walking around randomly without keeping the obstacles in mind. But when the actor_pos_publish node is called, the following happens:
1. The node subscribes to the global costmap provided by move_base
2. For each actor we choose a random point in the map with zero cost
3. For each actor we generate the path using the A* algorithm
4. Each generated path is published to a topic corresponding to the actor
5. Each actor plugin subscribes to this topic and tries to follow that path



https://user-images.githubusercontent.com/37510173/146939479-461cbed3-16f5-4ebe-8e87-fb14f64ad258.mp4



There are two packages in the repo, the actor_move folder is for the gazebo plugin for the actors and the actor_pos_publish folder is for the package that computes the paths of the actors and publishes them.

1. actor_pos_publish package:
```
Subscribes to: 
1. /gazebo/model_states (gazebo_msgs::ModelStates) 
2. /move_base/global_costmap/costmap (nav_msgs::OccupancyGrid) 

Publishes to:
1. /actor{i}/target (nav_msgs::Path) 
```

2. actor_move Gazebo plugin:
```
Subscribes to: 
1. /actor{i}/target (nav_msgs::Path) 
```
## 0. Important Files
1. [Actor plugin](https://github.com/YasinSonmez/Gazebo-Map-Actor-Plugin/blob/main/actor_move/MapActorPlugin.cc)
2. [Actor Positon Publisher Node](https://github.com/YasinSonmez/Gazebo-Map-Actor-Plugin/blob/main/actor_pos_publish/src/actor_pos_publish_node.cpp)
3. [A* Algorithm Implementation](https://github.com/YasinSonmez/Gazebo-Map-Actor-Plugin/blob/main/actor_pos_publish/src/plan_utils.cpp)
## 1. Setup
### 1.1 Actor Move Plugin
To build the gazebo plugin please follow the following steps. From the actor_move directory:
```
$ mkdir build
$ cd build
$ cmake ../
$ make
```
After that, a library named "libMapActorPlugin.so" will be generated in the build directory. Please update the reference path of "libMapActorPlugin.so" in the xxx_dynamic.world files in the gazebo_world/world directory before you use the dynamic world models. For example, open office02_dynamic.world and use "ctrl+F" to find "libMapActorPlugin.so". Then, replace the value of "filename" with the absolute path of "libMapActorPlugin.so" in your build directory of actor_move. Each animated actor needs to call this plugin. Therefore, please check all the reference paths of this plugin in the dynamic world models.

## 2. Using Actor Position Publisher Node
1. Launch your .launch file or the example .launch file from actor_pos_publish/launch directory:
```
roslaunch actor_pos_publish office02_dynamic_map.launch
```
2. Run the actor position publisher node:
```
rosrun actor_pos_publish actor_pos_publish_node
```
3. Now you should see the actors planning using the map in rviz

# References
1. https://github.com/bach05/gazebo-plugin-autonomous-actor
2. https://github.com/NKU-MobFly-Robotics/local-planning-benchmark
3. https://github.com/NKU-MobFly-Robotics/p3dx
