# ROS Workspace Code

This folder contains the ROS control and attack code. The control code generates random points which are used by the robot as goals. It reaches its goal using a PID angle controller. The attack code implements three attacks and each attack can be run by uncommenting the relevant code. This folder also contains a bag to numpy file which converts the bag files to numpy files.

## Prerequisits

This code requires you to have ROS installed.

## Running the code

The attack or control code can be run using:

```
$ rosrun robot_attack send_fake_messages.py
$ rosrun robot_drive drive_to_goal.py
```

The bag to numpy file can be run using:

```
$ rosrun bag_to_numpy convert.py "savefile.npy"
```

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)
