This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).
## Team
Capstone project for implemented by our team of 4 people, listed below:

|**Name**|**E-Mail**|**Role**|
|:-------|----------|---------:|
| Maxim Marant | maxim_marant@web.de | Team Lead|
| Polina Latypova | p_latypova@mail.ru | Team Member 1 |
| Róbert Mikulás | rmikulas@gmail.com | Team Member 2 |
| Géraud Martin-Montchalin | geraudmartinmontchalin@gmail.com | Team Member 3 |


## Project Overview
The capstone project of the Udacitys "Self-Driving Car Engineer" Nanodegree Program consist of 3 main parts: **Perception**, **Planning** and **Control**. Below you can find general approach for each part:

![](https://video.udacity-data.com/topher/2017/September/59b6d115_final-project-ros-graph-v2/final-project-ros-graph-v2.png)


#### Perception
Perception is based on two ROS Nodes " Traffic Light Detection" and "Obstacle Detection". Since the highway track in the simulator and also in real life to not have any obstacles, we were concetrated on the correct prediciton of the traffic light. For the traffic light detection we have to implement two modules *tl_detector* and *tl_classifier*. For the *tl_detector* a lot of information were provided during the walkthrough so it could be easily implemented. For the classification there was no requirement from udacity. Literature research and also reverse engineering of the "Autoware" code showed that the best way to implement the classifier is to use the tensorflow object dection API. Autoware has used [SSD Inception V2 model](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) for their project and this was also the way which we have followed in our capstone project for the light detection in the simulator. The usage of the pre trained API and also dataset collected from the udacity simulator was provided by [Alex Lechner](https://github.com/alex-lechner/Traffic-Light-Classification) and described in this [Medium Article](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62). For the real-world detection of the lights in the provided ".bag" file, we saw the the SSD inception model was not very accurate. So we've tried several other pretrained API models and could reach the best result with the "Faster RCNN" model. This model has a good accuracy but is pretty slow so we've decided only to use it for the realworld and not for the simulator.
Both models for the simulator and for the realworld were trained for 20,000 steps. The model can detect the traffic lights correctly in the simulator and show the results in the command line.

#### Planning 
Planning module based on "Waypoint Loader" and "Waypoint Updater" ROS nodes. Waypoint loader loads the track, and Waypoint updater updates the track: according to the current pose of the car (/current_pose topic) it calculates the closest point and formes a set of certain number of waypoints from closest waypoint to closest waypoint + LOOKAHEAD_WPS. 
It also implements method decelerate_waypoints which allows a car to slow down if an obstacle (in our case - traffic light) is detected ahead.

#### Control
Control module is based on "Waypoint follower" and "Twist controller" ROS packages. Waypoint follower implements a well-known and popular Pure Pursuit method to follow a set of waypoints. It sends messages to DBW node (from Twist controller package) through /twist_cmd topic. DBW node using Twist controller forms steering, brake and throttle commands. Twist controller uses simple PID controller to form throttle control commands, yaw controller to form steering commands, and calculates brake command on the base of throttle, current velocity and linear velocity values.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/maxim-mar/P9---Capstone-.git
```

2. Install python dependencies
```bash
cd P9---Capstone-
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd P9---Capstone-/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
