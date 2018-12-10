# Bag Files

The following folder contains the generated bag files of the control and attack code. Bag files which only contain control code are kept in the "Clean Runs" folder. Bag files which contain attack messages are stored in AttackRuns. 

## Original Data

This folder contains the original data, and no extra downloads are necessary. Each bag file is roughly 20 seconds worth of data. The data was created using ROS kinetic.

## Creating the data

The data can be created using standard ROS commands. During the execution of the control or attack code run the following command in an open terminal:

```
rosbag record -a
```

To record exactly 20 seconds of data run the following command:

```
rosbag record -a --duration=20
```

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)
