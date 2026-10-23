# Description

This ROS 2 package enables the Unitree Go2 robot to follow a person. It uses a PID controller to adjust the robot's forward motion and yaw, keeping the person centered in the camera image.

We use a customized Go2 ROS 2 SDK with a fix for the new firmware.

Original repo: https://github.com/abizovnuralem/go2_ros2_sdk

Applied fix: https://github.com/abizovnuralem/go2_ros2_sdk/issues/238#issuecomment-4569824976

# Architecture

The camera is read using `gst_to_image`, which runs as the `usb_camera` node and publishes `/usb_camera/image_raw`. This image is used by YOLO for object detection and tracking, and it is also sent as a UDP stream using `image_to_gst`.

After detections and tracking are performed, the `target_tracker_node` selects which person to follow. The selected detection is then sent to the `follower_node`.

The `follower_node` uses the position of the detection in the image to control the robot. It applies a PID controller to rotate the robot (yaw) and move it forward/backward so that the person stays centered in the image and maintains a target width, which corresponds to distance.

# Packages

The application is split into the following ROS 2 packages:

| Package | Language | Purpose |
|---------|----------|---------|
| `bringup` | Launch only | Top-level launch files; starts everything (nodes + Foxglove Bridge) |
| `gst_to_image` | C++ | Camera source: reads the device via a GStreamer pipeline; runs as the `usb_camera` node, publishes `/usb_camera/image_raw` |
| `yolo` | C++ | TensorRT YOLO inference, publishes `/yolo/detections` (Detection2DArray) |
| `tracker` | Python | Object tracking (BYTETracker/BOTSORT via Ultralytics), publishes `/tracker/detections` with track IDs |
| `follower` | Python | Contains two nodes: `target_tracker_node` (selects which person to follow) and `follower_node` (PID control) |
| `osd` | C++ | Draws Foxglove ImageAnnotations (bounding boxes, overlays) onto the image |
| `image_to_gst` | C++ | Pushes image frames into a GStreamer pipeline for UDP H.264 streaming |
| `dog_msgs` | ROS msgs | Custom messages: `Bbox2D` (normalized bbox), `DetectionStats` (FPS/latency) |
| `shared_qos` | Python | Shared QoS profiles used by the Python nodes |
| `go2_ros2_sdk` | Python | Customized Go2 ROS 2 SDK (WebRTC communication with the robot) |

# Get your AES-128 key

Instruction: https://github.com/legion1581/unitree_webrtc_connect#aes-128-key-data23-g1--151--go2--1115

# Setup
In the docker-compose.yml you can define some settings

Set where the UDP video stream should be sent.
```
- VIDEO_DESTINATION_IP=192.168.50.178
```

Before running the Docker container, set the key for Go2 on the host machine.
```
export GO2_AES_128_KEY=XXXXXX
```

### Set static IP
You need to set a static IP for ethernet, so we can communicate with the robot via ethernet cable. This IP should be in the same subnet as **192.168.123.161**

Source: https://support.unitree.com/home/en/developer/Quick_start (Network Configuration
)

```
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.123.222/24
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
```

### Working without a local WiFi network

In order to operate the robot without a local WiFi network, connect both the Jetson and your own PC to the robot's hotspot. The system remains fully functional in this configuration.


# Building
We keep the dependencies in a separate Docker image because installing them takes time. When we only change the code, we don't want to install the dependencies again with every build.

> **Note:** it is recommended to perform the first run with internet access available, since yolo models need to be downloaded. Subsequent runs work fine offline.

Build dependencies
```
docker compose build base
```

Build the code
```
docker compose build runtime
```

Run the code
```
docker compose up runtime --remove-orphans
```

-d can be added to run in detached mode
```
docker compose up runtime -d --remove-orphans
```

# Enabling the tracking functionality

The robot will not detect anyone or move until you enable the follower.

Enable following by calling the `/follower/enable` service (`std_srvs/SetBool`) with `data: true`:

Call this service from a **Service Call** panel in Foxglove Studio (service `/follower/enable`, type `std_srvs/SetBool`, request `{ "data": true }`).

You can also run it from inside the container:
```
docker compose exec runtime bash
source /opt/ros/humble/setup.bash

ros2 service call /follower/enable std_srvs/srv/SetBool "{data: true}"

```
Stop following by calling the same service with `data: false`:
```
ros2 service call /follower/enable std_srvs/srv/SetBool "{data: false}"

```

# Useful commands

### Change power mode to MAX
```
sudo nvpmodel -m 2
```

### Launching stream (on other PC)
```
gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" ! rtpjitterbuffer latency=50 ! rtph264depay ! h264parse ! avdec_h264 max-threads=4 ! videoconvert ! autovideosink sync=false
```

### Disable GUI on Jetosn to save RAM
```
sudo systemctl set-default multi-user.target
sudo reboot
```

### Enable GUI
```
sudo systemctl set-default graphical.target
sudo reboot
```
