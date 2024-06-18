# ROS2 Ouster Drivers

These are an implementation of ROS2 drivers for the Ouster lidar. This includes all models of the OS-x from 16 to 128 beams running the firmware 2.2-2.4. This is a legacy driver for those deploying within their applications. For new applications, we recommend checking out [Ouster's newly supported ROS 2 driver](https://github.com/ouster-lidar/ouster-ros) which we have also validated works well.

You can find a few videos looking over the sensor below. They both introduce the ROS1 driver but are extremely useful references regardless:

OS-1 Networking Setup      |  OS-1 Data Overview
:-------------------------:|:-------------------------:
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/92ajXjIxDGM/0.jpg)](http://www.youtube.com/watch?v=92ajXjIxDGM) | [![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/4VgGG8Xe4IA/0.jpg)](http://www.youtube.com/watch?v=4VgGG8Xe4IA)


## Documentation

Documentation can be generated using Doxygen.

Run `doxygen` in the root of this repository. It will generate a `/doc/*` directory containing the documentation. Entrypoint in a browser is `index.html`.

## Design

See design doc in `design/*` directory [here](ros2_ouster/design/design_doc.md).

## ROS Interfaces

<center>

| Topic                | Type                    | Description                                      |
|----------------------|-------------------------|--------------------------------------------------|
| `scan`               | sensor_msgs/LaserScan   | 2D laser scan of the 0-angle ring                |
| `range_image`        | sensor_msgs/Image       | Image of the range values from the sensor        |
| `intensity_image`    | sensor_msgs/Image       | Image of the Intensity values from the sensor    |
| `noise_image`        | sensor_msgs/Image       | Image of the noise values from the sensor        |
| `reflectivity_image` | sensor_msgs/Image       | Image of the reflectivity values from the sensor |
| `points`             | sensor_msgs/PointCloud2 | 3D Pointcloud generated from a 360 rotation      |
| `imu`                | sensor_msgs/Imu         | IMU values at transmission rate                  |

| Service           | Type                    | Description                       |
|-------------------|-------------------------|-----------------------------------|
| `reset`           | std_srvs/Empty          | Reset the sensor's connection     |
| `GetMetadata`     | ouster_msgs/GetMetadata | Get information about the sensor. A optional filepath can be specified to save the metadata to a local file.  |

| Parameter                | Type    | Description                                                                                                 |
|--------------------------|---------|-------------------------------------------------------------------------------------------------------------|
| `lidar_ip`               | String  | IP or hostname of lidar (ex. 10.5.5.87, os1-serialno.local)                                                 |
| `computer_ip`            | String  | IP or hostname of computer to get data (ex. 10.5.5.1) or broadcast (ex. 255.255.255.255) or if using the default driver, "" for automatic detection                   |
| `lidar_mode`             | String  | Mode of data capture, default `512x10`                                                                      |
| `imu_port`               | int     | Port of IMU data, default 7503                                                                              |
| `lidar_port`             | int     | Port of laser data, default 7502                                                                            |
| `sensor_frame`           | String  | TF frame of sensor, default `laser_sensor_frame`                                                            |
| `laser_frame`            | String  | TF frame of laser data, default `laser_data_frame`                                                          |
| `imu_frame`              | String  | TF frame of imu data, default `imu_data_frame`                                                              |
| `ethernet_device`        | String  | An ethernet device (e.g. eth0 or eno1) on which the Tins driver will listen for packets. Note that this is only a parameter for the Tins driver, and is only specified in the config file for that driver. |                              |
| `use_system_default_qos` | bool    | Publish data with default QoS for rosbag2 recording, default `False`                                        |
| `timestamp_mode`         | String  | Method used to timestamp measurements, default `TIME_FROM_INTERNAL_OSC`                                     |
| `os1_proc_mask`          | String  | Mask encoding data processors to activate, default <code>IMG &#124; PCL &#124; IMU &#124; SCAN</code> |
| `pointcloud_filter_zero_points` | bool | Reduce pointcloud size by omitting (0, 0, 0) points, default `False`. If used, will make the PC2 unstructured.     |


</center>

Note: TF will provide you the transformations from the sensor frame to each of
the data frames.

### Timestamp Modes

Referring to the parameter table above, the `timestamp_mode` parameter has four
allowable options (as of this writing). They are: `TIME_FROM_INTERNAL_OSC`,
`TIME_FROM_SYNC_PULSE_IN`, `TIME_FROM_PTP_1588`, `TIME_FROM_ROS_RECEPTION`. A
description of each now follows.

#### `TIME_FROM_INTERNAL_OSC`

Use the LiDAR internal clock. Measurements are time stamped with ns since
power-on. Free running counter based on the sensor’s internal oscillator. Counts
seconds and nanoseconds since sensor turn on, reported at ns resolution (both a
second and nanosecond register in every UDP packet), but min increment is on
the order of 10 ns. Accuracy is +/- 90 ppm.

#### `TIME_FROM_SYNC_PULSE_IN`

A free running counter synced to the `SYNC_PULSE_IN` input counts seconds (# of
pulses) and nanoseconds since sensor turn on. If `multipurpose_io_mode` is set to
`INPUT_NMEA_UART` then the seconds register jumps to time extracted from a NMEA
`$GPRMC` message read on the `multipurpose_io` port. Reported at ns resolution
(both a second and nanosecond register in every UDP packet), but min increment
is on the order of 10 ns. Accuracy is +/- 1 s from a perfect `SYNC_PULSE_IN`
source.

#### `TIME_FROM_PTP_1588`

Synchronize with an external PTP master. A monotonically increasing counter
that will begin counting seconds and nanoseconds since startup. As soon as a
1588 sync event happens, the time will be updated to seconds and nanoseconds
since 1970. The counter must always count forward in time. If another 1588 sync
event happens the counter will either jump forward to match the new time, or
slow itself down. It is reported at ns resolution (there is both a second and
nanosecond register in every UDP packet), but the minimum increment
varies. Accuracy is +/- <50 us from the 1588 master.

#### `TIME_FROM_ROS_RECEPTION`

The sensor will run in `TIME_FROM_INTERNAL_OSC` time mode but data are stamped with the ROS time when they are received. The inherent latency
between when the data were sampled by the LiDAR and when the data were received
by this ROS node is not modelled. This approach may be acceptable to get up and
running quickly or for static applications. However, for mobile robots,
particularly those traveling at higher speeds, it is not recommended to use
this `timestamp_mode`. When running in this mode, the on-LiDAR `timestamp_mode`
will not be set by this driver.

### Parameterizing the Active Data Processors

The `os1_proc_mask` parameter is set to a mask-like-string used to define the
data processors that should be activated upon startup of the driver. This will
determine the topics that are available for client applications to consume. The
*de facto* reference for these values are defined in
[processor_factories.hpp](ros2_ouster/include/ros2_ouster/processors/processor_factories.hpp). It
is recommended to only use the processors that you require for your application.

The available data processors are:

- **IMG** Provides 8-bit image topics encoding the noise, range, intensity, and
  reflectivitiy from a scan.
- **PCL** Provides a point cloud encoding of a LiDAR scan
- **IMU** Provides a data stream from the LiDAR's integral IMU
- **SCAN** Provides a 2D LaserScan from the closest to 0-degree azimuth ring

To construct a valid string for the `os1_proc_mask` parameter, join the tokens
from above (in any combination) with the pipe character (`|`). For example,
valid strings include but are not limited to: `IMG|PCL`, `IMG|PCL|IMU`, `PCL`,
etc. The default value is `IMG|PCL|IMU|SCAN`.

More details about data processors in the driver is provided in the [Additional
Lidar Processing](#additional-lidar-processing) section below.

## Extensions

This package was intentionally designed for new capabilities to be added. Whether that being supporting new classes of Ouster lidars (sensor-custom, OS2, ...) or supporting new ways of processing the data packets.

### Additional Lidar Processing
It can be imagined that if you have a stream of lidar or IMU packets, you may want to process them differently. If you're working with a high speed vehicle, you may want the packets projected into a pointcloud and published with little batching inside the driver. If you're working with pointclouds for machine learning, you may only want the pointcloud to include the `XYZ` information and not the intensity, reflectivity, and noise information to reduce dimensionality.

In any case, I provide a set of logical default processing implementations on the lidar and IMU packets. These are implementations of the `ros2_ouster::DataProcessorInterface` class in the `interfaces` directory. To create your own processor to change the pointcloud type, buffering methodology, or some new cool thing, you must create an implementation of a data processor.

After creating your implementation, that will take in a `uint8_t *` of a data packet and accomplish your task, you will need to create a factory method for it in `processor_factories.hpp` and add it to the list of processors to be created in the `createProcessors` method.

I encourage you to contribute back any new processor methods to this project! The default processors will buffer 1 full rotation of data of the pointcloud and publish the pointcloud with the X, Y, Z, range, intensity, reflectivity, ring, and noise information. It will also buffer a full rotation and publish the noise, intensity, and reflectivity images. Finally, it will publish the IMU data at transmission frequency.

Some examples:
- If you wanted the points at transmission frequency to reduce aliasing
- Different types of pointclouds published containing a subset or additional information.
- If you wanted the information in another format (ei 1 data image with 3 channels of the range, intensity, and noise)
- Downsample the data at a driver level to only take every `N`th ring.

### Additional Lidar Units
To create a new lidar for this driver, you only need to make an implementation of the `ros2_ouster::SensorInterface` class and include any required SDKs. Then, in the `driver_types.hpp` file, add your new interface as a template of the `OusterDriver` and you're good to go.

You may need to add an additional `main` method for the new templated program, depending if you're using components. If it uses another underlying SDK other than `sensor` you will also need to create new processors for it as the processors are bound to a specific unit as the data formatting may be different. If they are the same, you can reuse the `sensor` processors.

## Lifecycle

This ROS2 driver makes use of Lifecycle nodes. If you're not familiar with lifecycle, or managed nodes, please familiarize yourself with the [ROS2 design](https://design.ros2.org/articles/node_lifecycle.html) document on it.

The lifecycle node allow for the driver to have a deterministic setup and tear down and is a new feature in ROS2. The launch script will need to use a lifecycle node launcher to transition into the active state to start processing data.

## Component

This ROS2 driver makes use of Component nodes. If you're not familiar with component nodes please familiarize yourself with the [ROS2 design](https://index.ros.org/doc/ros2/Tutorials/Composition) document on it.

The component node allow for the driver and its processing nodes to be launched into the same process and is a new feature in ROS2. This allows the sensor and its data clients to operate without serialization or copying between nodes sharing a memory pool.

There's a little work in ROS2 Eloquent to launch a component-lifecycle node using only the roslaunch API. It may be necessary to include the Ouster driver in your lifecycle manager to transition into the active state when loading the driver into a process as a component. Example using the ROS2 component manager:

```bash
# component manager for dynamic loading (also may be done through launch)
ros2 run rclcpp_components component_container
# load this component
ros2 component load /ComponentManager ros2_ouster ros2_ouster::OS1Driver
# Set parameters
ros2 param set OusterDriver lidar_ip 10.5.5.86
ros2 param set OusterDriver computer_ip 10.5.5.1
# transition to configuring lifecycle stage
ros2 lifecycle set OusterDriver 1
# transition to active lifecycle stage (will now stream data)
ros2 lifecycle set OusterDriver 3
```

# Ouster Messages

A message `Metadata` was created to describe the metadata of the lidars. In addition the `GetMetadata` service type will be used to get the metadata from a running driver.


# Setup and Networking

Ouster gives you some tools to set up a direct connection to the sensor from you computer. I'd argue these are a bit obtuse and they should really provide a set of scripts to set this up automatically as a daemon. However, since I am also using this as a development tool, I don't want this always running in the background on my machines so I provide the directions below to setup the network connection.

### One time setup with IPv4

These are bash commands in Linux to setup the connection. These steps only need to happen the first time you set up the laser. After the first time, when you establish the network connection to the sensor, you can just select this created network profile. **Ensure the sensor is powered off and disconnected at this point.**

The `[eth name]` is the nework interface you're connecting to. On older Linux systems, that's `eth0` or similar. In newer versions, its `enp...` or `enx...` when you look at the output of `ifconfig`.

```bash
 ip addr flush dev [eth name]
 ip addr show dev [eth name]
```

The output you see from `show` should look something like `[eth name] ... state DOWN ...`. Its only important that you see `DOWN` and not `UP`. Next, lets setup a static IP address for your machine so you can rely on this in the future. Ouster uses the 10.5.5.* range, and I don't see a compelling reason to argue with it.

```bash
sudo ip addr add 10.5.5.1/24 dev [eth name]
```

Now, lets setup the connection. At this point you may now plug in and power on your sensor.

```bash
sudo ip link set [eth name] up
sudo addr show dev [eth name]
```

The output you see from `show` should look something like `[eth name] ... state UP ...`. Its only important that you see `UP` now and not `DOWN`. At this point, you've setup the networking needed for the one time setup.

### Connection with IPv4

We can setup the network connection to the sensor now with the proper settings. Note: This command could take up to 30 seconds to setup, be patient. If after a minute you see no results, then you probably have an issue. Start the instructions above over. Lets set up the network

```bash
sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i [eth name] --bind-dynamic
```

Instantly you should see something similar to:

```bash
dnsmasq: started, version 2.75 cachesize 150
dnsmasq: compile time options: IPv6 GNU-getopt DBus i18n IDN DHCP DHCPv6 no-Lua TFTP conntrack ipset auth DNSSEC loop-detect inotify
dnsmasq-dhcp: DHCP, IP range 10.5.5.50 -- 10.5.5.100, lease time 1h
dnsmasq-dhcp: DHCP, sockets bound exclusively to interface enxa0cec8c012f8
dnsmasq: reading /etc/resolv.conf
dnsmasq: using nameserver 127.0.1.1#53
dnsmasq: read /etc/hosts - 10 addresses
```

You need to wait until you see something like:

```bash
dnsmasq-dhcp: DHCPDISCOVER(enxa0cec8c012f8) [HWaddr]
dnsmasq-dhcp: DHCPOFFER(enxa0cec8c012f8) 10.5.5.87 [HWaddr]
dnsmasq-dhcp: DHCPREQUEST(enxa0cec8c012f8) 10.5.5.87 [HWaddr]
dnsmasq-dhcp: DHCPACK(enxa0cec8c012f8) 10.5.5.87 [HWaddr] os1-SerialNumXX
```

Now you're ready for business. Lets see what IP address it's on (10.5.5.87). Lets ping it

```bash
ping 10.5.5.87
```

and we're good to go!

### Using IPv6 with link local

Instead of having to configure `dnsmasq` and static addresses in the previous section, you can use link local IPv6 addresses.

1. Find the link local address of the Ouster. With avahi-browse, we can find the address of the ouster by browsing all non-local services and resolving them.
```bash
$ avahi-browse -arlt
+   eth2 IPv6 Ouster Sensor 992109000xxx                    _roger._tcp          local
+   eth2 IPv4 Ouster Sensor 992109000xxx                    _roger._tcp          local
=   eth2 IPv6 Ouster Sensor 992109000xxx                    _roger._tcp          local
   hostname = [os-992109000xxx.local]
   address = [fe80::be0f:a7ff:fe00:2861]
   port = [7501]
   txt = ["fw=ousteros-image-prod-aries-v2.0.0+20201124065024" "sn=992109000xxx" "pn=840-102144-D"]
=   eth2 IPv4 Ouster Sensor 992109000xxx                    _roger._tcp          local
   hostname = [os-992109000xxx.local]
   address = [192.168.90.2]
   port = [7501]
   txt = ["fw=ousteros-image-prod-aries-v2.0.0+20201124065024" "sn=992109000xxx" "pn=840-102144-D"]

```
As shown above, on interface `eth2`, the ouster has an IPv6 link local address of `fe80::be0f:a7ff:fe00:2861`.

To use link local addressing with IPv6, the standard way to add a scope ID is appended with a `%` character like so in `sensor.yaml`. Automatic detection for computer IP address can be used with an empty string.
```bash
lidar_ip: "fe80::be0f:a7ff:fe00:2861%eth2"
computer_ip: ""
```

Note that this feature is only available with the default driver version, configured by `driver_config.yaml`. When running the Tins-based driver (see the following sections), both the LiDAR and computer IP address must be specified in `tins_driver_config.yaml`.

### Usage with the default driver

Now that we have a connection over the network, lets view some data. After building your colcon workspace with this package, source the install space, then run:

```bash
ros2 launch ros2_ouster driver_launch.py
```

Make sure to update your parameters file if you don't use the default IPs (10.5.5.1, 10.5.5.87). You may also use the `.local` version of your ouster lidar. To find your IPs, see the `dnsmasq` output or check with `nmap -SP 10.5.5.*/24`.
An alternative tool is [avahi-browse](https://linux.die.net/man/1/avahi-browse): 

```bash
avahi-browse -arlt
```

Now that your connection is up, you can view this information in RViz. Open an RViz session and subscribe to the points, images, and IMU topics in the laser frame. When trying to visualize the point clouds, be sure to change the Fixed Frame under Global Options to "laser_data_frame" as this is the default parent frame of the point cloud headers.

When the driver configures itself, it will automatically read the metadata parameters from the Ouster. If you wish to save these parameters and use them with captured data (see the next section) then you can save the data to a specific location using the `getMetadata` service that the driver offers. To use it, run the driver with a real Ouster LiDAR and then make the following service call:

```bash
ros2 service call /ouster_driver/get_metadata ouster_msgs/srv/GetMetadata "{metadata_filepath: "/path/to/your/metadata.json"}"
```

The driver will then save all the required metadata to the specified file and dump the same metadata as a JSON string to the terminal. Alternatively, the service can be called without specifying a filepath (see below) in which case no file will be saved, and the metadata will still be printed to terminal. Copying this string and manually saving it to a .json file is also a valid way to generate a metadata file.  

```bash
ros2 service call /ouster_driver/get_metadata ouster_msgs/srv/GetMetadata
```

Have fun!

### Usage with Tins-based driver

If you want to use the driver to read data from a pcap file, you can use the `Tins`-based driver. To do this, open the `tins_driver_config.yaml` file and edit the following parameter:

* `ethernet_device`: Change this to a working ethernet device on your computer that you plan to replay data through (e.g. "eth1").

You can run the Tins driver with the command below. This will use the default `ouster_os0128_1024_metadata.json` file:

```bash
ros2 launch ros2_ouster tins_driver_launch.py
```

Alternatively, you can change the metadata being used by specifying the metadata filepath as shown below. You can generate a metadata file using the `getMetadata` service as shown in the previous section. Or you can use one of the example metadata files provided, which come from an Ouster OS0-128 in either 1024x10 or 2048x10 mode.

```bash
ros2 launch ros2_ouster tins_driver_launch.py metadata_filepath:=/path/to/metadata.json
```

After launching the driver, in a new terminal, you can replay a pcap file of recorded ouster data using the following command (as an example):

```bash
sudo tcpreplay --intf1=eth1 saved_ouster_data.pcap
```

You may need to run this command with `sudo`. Note that this driver version will also work with a live Ouster sensor, provided the data is coming into the correct ethernet device, and the parameters in the metadata file match those of the sensor. However it is recommended that you run the default drive with a real sensor, as this will guarantee that the metadata settings are correct.
