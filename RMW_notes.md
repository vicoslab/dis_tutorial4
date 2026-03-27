# ROS Middleware (RMW) Guide

ROS 2 is built on top of a data distribution abstraction which provides topic discovery, message serialization and network transport. This layer is usually handled by a [Data Distribution Service](https://www.omg.org/omg-dds-portal/) (DDS) such as CycloneDDS and FastDDS, however a RMW can be anything that conforms to the specification (e.g. [rmw_email](https://github.com/christophebedard/rmw_email)) and can act as a drop-in replacement. Not all RMWs are made equal however, and when something doesn't work correctly in ROS 2, the problem is usually the DDS.

![RMW Table](figs/chart.png)
*ROS 2 Message Stack*

When a node is initialized it will use the rmw API to try and connect to other nodes using the specified RMW (you can check what's currently being used with `echo $RMW_IMPLEMENTATION`) and declare its publishers and subscribers. This can involve a central server that runs as the `ros2 daemon` service, or be done on a peer-to-peer basis. 

A consequence of using DDS-based RMWs are Quality of Service (QoS) policies which can set topics to be reliable (like TCP), as best-effort (like UDP), or persistent. More on [QoS settings](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html#id6). It is important that a publisher and subscriber use a compatible QoS, otherwise communication cannot take place.

[More advanced info](https://docs.ros.org/en/jazzy/Concepts/Advanced/About-Internal-Interfaces.html).

### Environment variable config

If you want to use multiple computers in parallel, your computers should have the appropriate environment variables set up.  For this, you can set a `ROS_DOMAIN_ID` variable, so that ROS 2 will limit the visible nodes/topics only to those that are running under the same `ROS_DOMAIN_ID`. 

All RMWs support the `ROS_DOMAIN_ID` setting:
```bash
export ROS_DOMAIN_ID=<number from 0 to 99>
```

[iRobot's Create3 documentation](https://iroboteducation.github.io/create3_docs/setup/xml-config/#cyclonedds) is a good resource on RMW configuration.

------

### FastRTPS Env variables

In Jazzy, the preconfigured middleware is [FastDDS](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/DDS-Implementations/Working-with-eProsima-Fast-DDS.html), so one does not have to explicitly declare it. By default it attemps to connect to any other nodes using **simple discovery mode** on the network. That approach is generally not very reliable, so it's best to restrict node discovery to a single machine using:

```bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

It can also operate using a **discovery server** which acts as a central database of available nodes and topics, which can be more reliable for multi-machine configurations, but requires extensive configuration on both ends. More [info](https://fast-dds.docs.eprosima.com/en/3.x/fastdds/ros2/discovery_server/ros2_discovery_server.html).

### Zenoh Env variables

[Zenoh](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html) is a new and most promising RMW, designed for minimal overhead and tends to be the fastest. Unlike other RMWs, it currently requires explicitly running a router which relays traffic and handles discovery. After installing it, it can be enabled using:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Zenoh runs only on localhost by default. You can connect nodes directly to an existing zenohd router on another PC at e.g. 192.168.1.10:
```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.1.10:7447"]'
```

### Cyclone Env variables

[Cyclone](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html) is more compute intensive than FastDDS or zenoh, however it performs very reliably at multi-machine node discovery.

```bash
export CYCLONEDDS_URI='/home/rins/cyclonedds.xml'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The `cyclonedds.xml` config file can either declare which network interfaces Cyclone should use, or let it figure things out by itself:

```xml
<CycloneDDS>
   <Domain>
     <General>
        <Interfaces>
          <NetworkInterface autodetermine="true" />
        </Interfaces>
        <DontRoute>true</DontRoute>
    </General>
   </Domain>
</CycloneDDS>
```

```xml
<CycloneDDS>
   <Domain>
     <General>
        <Interfaces>
          <NetworkInterface name="eth0" />
          <NetworkInterface name="wlan0" />
        </Interfaces>
        <DontRoute>true</DontRoute>
    </General>
   </Domain>
</CycloneDDS>
```

It can also be set to a loopback interface which restricts it to only localhost:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo" />
      </Interfaces>
      <DontRoute>true</DontRoute>
    </General>
  </Domain>
</CycloneDDS>
```
In this case multicast would need to be additionaly enabled on lo, depending on your OS. 

For Ubuntu [see the autoware guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/enable-multicast-for-lo/).
