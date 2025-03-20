
# Tutorial 4: Some further information and help

#### Development of Intelligent Systems, 2024
This week you should be working on Task1. Here are some technical details to help you with the implementation.


### Environment variables to set
If you want to use multiple computers in parallel, your computers should have the appropriate environment variables set up.  For this, you can set a `ROS_DOMAIN_ID` variable, so that ROS2 will limit the visible nodes/topics only to those that are running under the same `ROS_DOMAIN_ID`. To do this, you need to choose a `<number>` between 0 and 101, and add the following line to the `.bashrc` file of all computers that you wish to be able to see each other (as far as ROS2 is concerned):
```
    export ROS_DOMAIN_ID=<number>
```
Alternatively, if you wish to only run ROS2 on a single computer, you can set:
```
    export ROS_LOCALHOST_ONLY=1
```
In the case when Ignition Gazebo fails to start, and you see repeated prints of `Requesting list of world names.`, setting the `IGN_IP` will probably help:
```
    export IGN_IP=127.0.0.1
```

### Saying Hello
As part of Task1, your robot needs to say something when it approaches a detected face. To do this, you can simply record an audio file, and use a module like `playsound` (that you install with `pip install playsound`). There are also other modules for playing sounds: `pydub`, `simpleaudio`, or using the `os` library and playing the sound with your system player.     
A more interesting approach, and one that will also prove useful in the future, is using a text-to-speech (TTS) generator. This is currently a very active research field, and you have many different options for TTS generators. There are simple ones, complex ones, there are those that run on-device, and those that run in the cloud, you can even try to train a deep model for imitating some voice. For our purposes, the quality of the generated voice does not matter, so do as you wish. One useful guide that you can use is: https://picovoice.ai/blog/on-device-text-to-speech-in-python/ .

## Coordinate transforms
Establishing the relative positions of the many components of a robot system is one of the most important tasks to ensure reliable positioning. Each of the components, such as map, robot, wheels, and different sensors has its own coordinate system and their relationships must be tracked continuously. The static relationships (such as between different static sensors) are relatively simple, consisting of only sequential rigid transformations that need to be computed only once. The dynamic relationships, such as the relative position of the robot to the world coordinates or the positions of the robot arm joints to the camera, however, must be estimated during operation.
You can install the tools for viewing the tf2 transforms as follows:
```
sudo apt install ros-humble-tf2-tools
```
Then, while your simulation is running, you can run `ros2 run tf2_tools view_frames` to record the tree of the tf2 transformations. This will create a .pdf representation of the tf2 tree that you can analyze.
You can also request the values of the specific transformation between two coordinate frames using the command:
```
ros2 run tf2_ros tf2_echo base_link map
```
This is important for placing the observed objects (faces, rings etc.) into the map, when they will be firstly detected in the coordinate frame of the robot (or, more specifically, in the coordinate frame of the sensor). You can use the scripts included in this tutorial to get you started. You can also work through the [tutorial for the tf2 transforms](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html): 

### Libraries for transformations
If you do not want to write your own transformations (e.q. Euler angles to Quaternions or vice versa) you can use the following libraries:

```
    sudo apt install ros-humble-tf-transformations
    sudo apt install ros-humble-turtle-tf2-py
    pip3 install transforms3d
```

Then, you can do stuff like:
```
    import tf_transformations
    q = tf_transformations.quaternion_from_euler(r, p, y)
    r, p, y = tf_transformations.euler_from_quaternion(quaternion)
```
### Using ROS bags
Those of you that can only work on the simulation in the lab, make use of the `ros2 bag` command line tool. It is a tool for recording all or some messages published. For example, you can run the simulation and drive to robot around the polygon, while recording the messages published (like the images from the camera). Then you can copy the `bag` file to another computer, play it, and work on face detection and clustering. The tutorial for `ros2 bag` is [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

### map_goals.py
In the demo package this week, you have a node that sends a navigation goal to the robot by clicking on the map. This demo package illustrates some ideas, like how you can read the map from the topic and convert it to a numpy image, how you can convert from pixel coordinates to real world coordinates and more. Download, build, test, and explore the code.

### transform_point.py
The other script in this package demonstrates how you can use the TF2 libraries to do transformations between frames. This node first creates a point that is 0.5m behind of the robot (in its header the frame_id is "base_link"). Then it looks up the transformation between the "base_link" and the "map" frames, and then applies the transformation to the point so it is transformed in the "map" frame. Then it creates a marker from the point and publishes it, so you can see it in Rviz on the `/breadcrumbs`topic.