# Offline Feature Simulation

## Objective

Design and implement a system to simulate interactions between an IoT device (our scanner) and a cloud infrastructure. You can use actual hardware devices or docker containers.

## Core Features to Implement
1. **Data Logging**: The IoT device should log random data at regular intervals.

2. **Data Storage and Transfer:** Store data locally on the device when offline and automatically transfer it to the cloud upon reestablishing connectivity.


## Solution

### Approach

1. **Environment** 
As a first step, I have to set up the environment. I will set everything in a docker container, as it makes it easier to set and distribute the solution, making sure that the environment and the dependencies will work, regardless the system (there are obviously some limitations though).
2. **Middleware**
I chose to use ROS2 as it makes it easy to log and manipulate data coming from a sensor, and is the go-to approach for robotics/IoT solutions. I chose the Humble distribution as it has EOL 2027.
2. 


### Steps

1. Clone the repository
```
$ git clone https://github.boschdevcloud.com/Half-Dome/gpu-com-docker
$ mkdir ros2_ws
$ cd ros2_ws
```


## Build Image and Start Docker

* this scripts will do:
    * build the image
    * run image
        * with GPU
        * mount repo directory
    * **OR** connect to the already running container
```
$ ./start_docker.sh 
```

2. 