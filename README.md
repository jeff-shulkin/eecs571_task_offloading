# ROS2 Installation Guide

To install the proper TurtleBot4 packages for ROS2 on Ubuntu 24.04, follow the instructions below:

1. Update OS and packages prior to ROS installation

```
sudo apt-get update -y
sudo apt-get upgrade -y
```

2. Navigate to the turtlebot4\_ws directory, and run the turtlebot4\_setup.sh script.

```
cd turtlebot4_ws
./turtlebot4_setup.sh
``` 

# TODOs
## offload_client
- [ ] [IN PROGRESS] switch between offload and non-offload

## offload_server
- [ ] [TEST PENDING] nav2 final_pose ready callback
- [ ] [TEST PENDING] global server flag RESULT_READY
- [x] busy wait in the scheduler.execute()
- [ ] time the scheduler execution
- [ ] protect variables with mutexes
- [ ] [IN PRORGRESS] manage lifecycle of Nav2

## turtlbot4
- [ ] try to get internet on the turtlebot pi

## evaluation
- 
