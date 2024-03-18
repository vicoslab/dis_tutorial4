# Tutorial 4: The Turtlebot 4

#### Development of Inteligent Systems, 2024
This week you should be working on Task1. Here are some technical details to help you with the implementation.

## Laboratory laptops for running the simulation
As you know by know, the Turtlebot4 simulation is notoriously compute intensive, and requires a dedicated GPU in order to be able to run the simulation fast enough so that it does not break completely. For this purpose, we set up six laptops in the classroom.

![Laptops](figs/lab_laptop.jpg "Laptops")

Each team has a username for each laptop, so you can use whichever one you like. Set up a Git repository for your code, since it is NOT guaranteed that you will find the files on the laptop in the same state as you left them. If you do not have the password for your team, ask the assistant.

## Environment variables to set
As know, ROS2 uses a DDS system in order to manage the resources in a ROS2 graph. This means that all the nodes, and topics, and services on the same network can find eachother. This can be problematic when there are multiple people independently running ROS2 nodes, that should not communicate to each other. To fix this, we can set a `ROS_DOMAIN_ID` variable, so that our nodes will comunicate only nodes that are running under the same `ROS_DOMAIN_ID`. If you set up the same `ROS_DOMAIN_ID` on the laboratory laptops and your own laptop, you will be able to run the simulation on the lab laptos, and run other nodes on your own computers. To do this, you need choose a `<number>` between 0 and 101, and add the following line to the `.bashrc` file of any computer that you wish to be able to see eachother (as far as ROS2 is concearned):

```
    export ROS_DOMAIN_ID=<number>
```

Alternatively, if you will to only run ROS2 on a single computer, you can set:

```
    export ROS_LOCALHOST_ONLY=1
```

Additionally, Ignition Gazebo fails to start and you see repeated prints of `Requesting list of world names.`, setting the `IGN_IP` will probably help:

```
    export IGN_IP=127.0.0.1
```

## Saying Hello
As part of Task1, your robot needs to say something when it approaches a detected face. To do this, you can simply record an audio file, and use a module like `playsound` (that you install with `pip install playsound`). There are also other modules for playing sounds: `pydub`, `simpleaudio`, or using the `os` library and playing the sound with your system player.     
A more interesting apprach, and one that will also prove useful in the future, is using a text-to-speech (TTS) generator. This is currently a very active research field, and you have many different options for TTS generators. There are simple ones, complex ones, there are those that run on-device, and those that run in the cloud, you can even try to train a deep model for imitating some voice. For our purposes, the quality of the generated voice does not matter so do as you wish. One useful guide that you can use is: https://picovoice.ai/blog/on-device-text-to-speech-in-python/

## Libraries for transformations
If you do not want to write your own transformations (e.q. Euler angles to Quaternions or vice-versa) you can use the following libraries:

```
    sudo apt install ros-humble-tf-transformations
    pip3 install transforms3d
```

Then, you can do stuff like:
```
    import tf_transformations
    q = tf_transformations.quaternion_from_euler(r, p, y)
    r, p, y = tf_transformations.euler_from_quaternion(quaternion)
```

### Some demo functionalities

In the demo package this week, you have a node that sends a navigation goal to the robot by clicking on the map. This demo package illustrates some ideas, like how you can read the map from the topic and convert it to a numpy image, how you can convert from pixel coordinates to real world coordinates and more. Download, build, test, explore code.