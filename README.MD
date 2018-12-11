# Software Security - Final Project

This is my final software security project for my fall semester 2018 at the University of Virginia. This project is about assessing and defending against security threats in the robotic operating system (ROS). For this project, I design a segment of code which drives a robot to random goals while observing sensor data. I then created another piece of code which attacks the robot using different attack techniques. Finally, I use Keras to develop a recurrent neural network (RNN) which uses LSTM cells to classify between attack messages and standard messages.

I am able to achieve a 92% accuracy with this model, meaning that I would have been able to drop and ignore 92% of my created attack messages.

## Project Sections


* [BagFiles](./BagFiles/) - Contains the recorded bagfiles of the control and attack code used by the LSTM for classification.
* [Code](./Code/) - Contains both the LSTM and the ROS code.
* [Documents](./Documents) - Contains all documentation done for this project.
* [NumpyFiles](./NumpyFiles) - Contains the converted bagfiles, which are now saved as numpy files. Also contains code used to merge files into a single numpy file.
* [Videos](./Videos) - Contains the raw and final video results from the project.

## Video

[Software Security Final Project - Part 1](https://youtu.be/T-RW7DHarrI)

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)