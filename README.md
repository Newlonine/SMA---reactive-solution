# Multi-Agent System

This project is about the different behaviors of a multi-agent system. It was done as part of my M1 internship and was proposed by the INUIT team of the Lab-STICC, a research laboratory.
The purpose of this project is to implement a multi-agent system to explore an unknown environment by robots to find a target.
This repository has to be used with [Webots](https://cyberbotics.com/), an open-source robot simulator.

# Controllers
Multiple controllers are available to control the agents.

## epuck_cognitive_behavior
This controller is the most recent and is not yet finished. 

This controller uses the epuck_reactive_behavior_V4.c and a function called "compute_odometry" which allows an agent to calculate its position in the environment.

## epuck_reactive_behavior

There are 4 controllers for the reactive behavior of the agents.

 - **epuck_reactive_behavior**: First version of the reactive behavior, the target and robots communicate through a receiver and an emitter. If a robot picks up a signal from the target, it will steer towards it. It uses a simple obstacle avoidance and has to be used with a target that has **goal_emitter.c** controller.

 - **epuck_reactive_behavior_V2**: First version of the reactive behavior of robots to use a camera. It also uses a receiver to pick up a signal from the supervisor if the robot is within the target range.
 
 - **epuck_reactive_behavior_V3**: First version to detect other robots using the camera. If the robot does not see any objects (other robots and/or targets) it will circle around and as long it does not see any objects it will reduce its angular speed. If the robot sees 4 or more objects it will drive towards it. This version must be used with **supervisor_controller_V2.c**.
 
 - **epuck_reactive_behavior_V4**: The supervisor no longer has a communication with the robots. If the robot is in range of the target, it has a certain probability to stop. For the first 100 steps, the robot checks if a robot is in its viewpoint and if so, it spins around. This version must be used with **supervisor_controller_V3.c** or **supervisor_efficiency.c**.


## goal_emitter
This is the first controller for a target. For it to be detected, it needs to be equipped with an emitter and a receiver. In order to find this target the robots must use **epuck_reactive_behavior.c**. If the target has been found and receives a message from a robot finding it, it increases its emitting range for each.

## supervisor_controller
- **supervisor_controller_V2.c**: This supervisor places all the robots randomly. The robots must be in the world and have a DEF prior to that. The supervisor monitors the position of each robot and if one of them is in range of the target the supervisor sends a message to it. When all robots have found the target, the supervisor pauses the simulation. Can be used with **epuck_reactive_behavior_V2.c** or **epuck_reactive_behavior_V3.c**.

- **supervisor_controller_V3.c**: Same as **suprevisor_controller_V2.c** but without the emitter. The supervisor and the agents no longer communicate. Can only be used with **epuck_reactive_behavior_V4.c**.

- **supervisor_efficiency.c**: This supervisor calculates the efficiency of the simulation. The formula is: sum of the distance at step 0 of each robot and the target divided by the sum of the distance traveled by each robot.

# Protos
I created and modified some prototypes for my simulations. They are: 

- **Cible.proto**: a target with an emitter and a receiver.
- **CibleWithRange.proto**: a target that is recognizable by the object recognition of a camera.
- **E-puckCameraRecognition.proto**: a modified prototype of the e-puck that has object recognition.
- **E-puckCompass.proto**: modified prototype of the e-puck that has a compass.

