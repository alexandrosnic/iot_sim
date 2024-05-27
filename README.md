# Offline Feature Simulation

## Objective

Design and implement a system to simulate interactions between an IoT device (our scanner) and a cloud infrastructure. You can use actual hardware devices or docker containers.

## Core Features to Implement
1. **Data Logging**: The IoT device should log random data at regular intervals.

2. **Data Storage and Transfer:** Store data locally on the device when offline and automatically transfer it to the cloud upon reestablishing connectivity.


## Solution

### Approach
There are few design questions needed to be answered before we tackle the problem:
* **OS**
We will work on Ubuntu Jammy (22.04)
* **Environment** 
As a first step, I have to set up the environment. I will set everything in a docker container, as it makes it easier to set and distribute the solution, making sure that the environment and the dependencies will work, regardless the system (there are obviously some limitations though).
* **Middleware**
I chose to use ROS2 as it makes it easy to log and manipulate data coming from a sensor, and is the go-to approach for robotics/IoT solutions. I chose the Humble distribution as it has EOL 2027.
* **Dataset**
For data we will use camera's images from NVidia's (r2b_hope dataset)[https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023]. The data had been downloaded via docker.


### Steps

**1. Clone the repository**
```
$ git clone https://github.com/alexandrosnic/iot_sim.git
$ cd iot_sim
```

**2. Build image and start docker**

* This scripts will:
    * build the image
    * run image
        * with GPU
        * mount repo directory
    * **OR** connect to the already running container
```
$ ./start_docker.sh 
```

**3. Build the workspace**
Run
```
colcon build
source install/setup.bash
```

**4. Run the launch file**
```
cd src/iot_sim_launch/launch/
ros2 launch iot_simulation.launch.py
```

Or run each node independently:

**4.1. Run the rosbag**
```
ros2 bag play data/r2b_hope.db3 --loop --rate 0.5
```

**4.2. Visualize the data**

Open another terminal and connect to the docker
```
docker exec -it iot-simulation bash
```
Open Rviz2
```
rviz2 -d camera_vis.rviz
```

**4.3. Run the logger**

Open another terminal, connect to the docker, and then run the logger
```
docker exec -it iot-simulation bash
ros2 run iot_logger iot_logger
```

**4.4. Run the uploader**

Open another terminal, connect to the docker, and then run the uploader
```
docker exec -it iot-simulation bash
ros2 run iot_uploader iot_uploader
```

### Troubleshoot
1. In case there are issues with docker, make sure shell files have executable permissions:
```
chmod +x entrypoint.sh
```