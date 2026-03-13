
# Tutorial 4: Help & Info

#### Development of Intelligent Systems, 2026

This week you should be working on Task1. Here are some technical details to help you with the implementation.

## Coordinate Transforms

Establishing the relative positions of the many components of a robot system is one of the most important tasks to ensure reliable positioning. Each of the components, such as map, robot, wheels, and different sensors has its own coordinate system and their relationships must be tracked continuously. The static relationships (such as between different static sensors) are relatively simple, consisting of only sequential rigid transformations that need to be computed only once. The dynamic relationships, such as the relative position of the robot to the world coordinates or the positions of the robot arm joints to the camera, however, must be estimated during operation.
You can install the tools for viewing the tf2 transforms as follows:

```bash
sudo apt install ros-jazzy-tf2-tools
```

Then, while your simulation is running, you can run `ros2 run tf2_tools view_frames` to record the tree of the tf2 transformations. This will create a .pdf representation of the tf2 tree that you can analyze.
You can also request the values of the specific transformation between two coordinate frames using the command:
```
ros2 run tf2_ros tf2_echo base_link map
```
This is important for placing the observed objects (faces, rings etc.) into the map, when they will be firstly detected in the coordinate frame of the robot (or, more specifically, in the coordinate frame of the sensor). You can use the scripts included in this tutorial to get you started. You can also work through the [tutorial for the tf2 transforms](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html): 

### Libraries for Transformations
If you do not want to write your own transformations (e.q. Euler angles to Quaternions or vice versa) you can use the following libraries:

```bash
sudo apt install ros-jazzy-tf-transformations ros-jazzy-turtle-tf2-py
pip install transforms3d --user --break-system-packages
```

Then, you can do stuff like:
```python
import tf_transformations
q = tf_transformations.quaternion_from_euler(r, p, y)
r, p, y = tf_transformations.euler_from_quaternion(quaternion)
```

## Nodes

This week's demo nodes deal with mapping points between different coordinate systems. You can use these as the foundation for your own implementation of the tasks.

### map_goals.py

The node `map_goals.py` reads the pre-constructed map from topic `/map` and displays it in a new window. It then waits for user input. When the user clicks on a valid point in the map, the node sends a navigation goal to the robot. This node illustrates some ideas, like how you can read the map from the topic and convert it to a numpy image, how you can convert from pixel coordinates to real world coordinates and more. Build, run, and explore the code.

### transform_point.py

The node `transform_point.py` demonstrates how you can use the TF2 libraries to do transformations between frames. The node sets up the lookup to the coordinate system transformation tree, specifically the transform between frames `map` and `base_link` (the base of the robot). This allows us to define new points in the coordinate system of the robot, and express them in map coordinates. New markers are created at a distance of 0.5m behind the robot and published to the `/breadcrumbs` topic. If we published these without the appropriate transformation, they would always be located behind the robot. However, if we use the correct transformation, the robot will leave a trail of markers behind itself when driving around the map. You will use transformations such as these in your tasks on to place detected objects and faces into the map.

See [the documentation page](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html) for the available marker types and more info.

#### Quaternions

By now, you will probably have observed that the robot orientation in ROS2 is represented with 4 values (instead of the more intuitive 3 values). The reason is that the orientation is expressed with quaternions. Quaternions are one of the ways of representing rotation in 3 dimensions. Some of their advantages are conciseness (4 values are the minimum for 3d rotation), no singularities, and simple interpolation (for graphics). However, they are quite complex and unintuitive. One way of imagining quaternion rotation is to think of it as a rotation about an axis represented in 3 dimensions. 

Further reading regarding quaternions can be found in [ROS2 documentation](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html). You can also check out this [visualizer](https://quaternions.online/).

For easier development and debugging, you might want to transform the poses to a more intuitive representation, such as Euler angles (pitch, roll and yaw are rotations about the x, y, and z axis, respectively). Since we are dealing with a ground robot, the only applicable rotation is around the z axis (the yaw angle). Transform the robot's orientation to Euler angles and display the current orientation using an arrow marker.

You can read more about ROS2 marker types [here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html).

## Saying Hello

As part of Task1, your robot needs to say something when it approaches a detected face. To do this, you can simply record an audio file, and use a module like `playsound` (that you install with `pip install playsound`). There are also other modules for playing sounds: `pydub`, `simpleaudio`, or using the `os` library and playing the sound with your system player.     

A more interesting approach, and one that will also prove useful in the future, is using a text-to-speech (TTS) generator. This is currently a very active research field, and you have many different options for TTS generators. There are simple ones, complex ones, there are those that run on-device, and those that run in the cloud, you can even try to train a deep model for imitating some voice. 

For our purposes, the quality of the generated voice does not matter, so do as you wish. Some modern include (in roughly increasing order of quality and compute required):
- [espeak-ng](https://github.com/gooofy/py-espeak-ng)
- [Piper TTS](https://github.com/OHF-Voice/piper1-gpl)
- [Kitten TTS](https://github.com/KittenML/KittenTTS)
- [Kokoro TTS](https://github.com/nazdridoy/kokoro-tts)
- [Qwen3-TTS](https://github.com/QwenLM/Qwen3-TTS)

## Using ROS bags

Those of you that can only work on the simulation in the lab, make use of the `ros2 bag` command line tool. It is a tool for recording all or some messages published. For example, you can run the simulation and drive to robot around the polygon, while recording the messages published (like the images from the camera). Then you can copy the `bag` file to another computer, replay it there, and work on face detection and clustering. The tutorial for `ros2 bag` is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

## Communication between multiple computers in ROS2

If you want to use multiple computers so they will be able to see each others' ROS2 topics, there are several options, depending on the RMW used. You will have to set `export ROS_LOCALHOST_ONLY=0` and `export ROS_DOMAIN_ID=<id>` (number between 0 and 101) so RMW will only listen on the appropriate [ports](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

More in depth information regarding different RMW implementations is available [here](RMW_notes.md).
