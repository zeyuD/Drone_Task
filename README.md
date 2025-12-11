# Drone Tasks

## About

This repo is about the drone project.



## Current Progress
### 
- Generate a random maze grid and plan path using A* algorithm.
- Use ArUco Python package to detect markers with ID in a video.


## Arranged To Do List
- Realtime detection using webcam.
- Detect orientation and distance.


## To Be Implemented
- Integration to ROS2.


## Tips 


### Environment pre-requests

Install PX4-Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Install Micro-XRCE-DDS-Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```