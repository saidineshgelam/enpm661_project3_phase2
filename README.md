# enpm661_project3_phase2
# A* Algorithm for Differential Drive Robot
The A* pathfinding method for a differential drive robot (TurtleBot3 Waffle) in a simulated environment can be run using the instructions in this README. Taking into account the robot's non-holonomic restrictions, the code guides the robot from a start point to a goal point.

## Overview
The approach uses an 8-action set to meet differential driving restrictions while mimicking the robot's movement in a map environment. 

## Pre-requisites
Ensure you have the following installed:

- Python3
- ROS2 
- Gazebo
- OpenCV
- Numpy
- Matplotlib

## 
Part 01: 2D Implementation

### User Inputs
The script takes the following inputs from the terminal:

- Start Point Coordinates.
- Goal Point Coordinates. 
- Initial Orientation.
- Goal Orientation.
- Wheel RPMs: (RPM1, RPM2) for the robot's wheels.
- Clearance: The minimum distance between the robot and any obstacle.

### Example inputs

- Start Point Coordinates: (500, 1000)
- Goal Point Coordinates:  (5500, 750)
- Initial Orientation: 0
- Goal Orientation: 0
- RPM1: 60
- RPM2: 100
- Clearance: 150

### Running the Code
Execute the script using the following command in your terminal:


## Part 02: Gazebo Simulation
### Setup
Follow the instructions from this repository to set up the ROS2 workspace with the TurtleBot3 Gazebo package.

Launch the Simulation
Run the Gazebo environment and ROS2 nodes using:

## Team Members
Member 1: Sai Dinesh Gelam, sgelam, 120167140
Member 2: Gowtham VMS Chintalapati, gowch15, 120239878


## Repository Link
Link to GitHub Repository: 


## Videos:
Part 01 Simulation link: 

Part 02 Gazebo Simulation link:

