# Bug-Algorithms-Simulation

This is the course final project of the *Fundamental of Robotics - Fall 2021* course at Shahid Beheshti University. This repository contains the Python implementation of Bug algorithms and simulation with 3-wheel omnidirectional robot in Webots simulation software.

## Introduction to Bug algorithms
The Bug algorithm is perhaps the simplest obstacle-avoidance algorithm one could imagine. The basic idea is to follow the contour of each obstacle in the robot’s way and thus circumnavigate it. A number of variations and extensions of the Bug algorithm exist, otherwise we have implemented the three main ones in Python.

- **Bug 0:** With Bug0, the robot begins to follow the object’s contour, but departs immediately when it is able to move directly toward the goal. In general this algorithm does not guarantees that the robot reach the goal.

- **Bug 1:**  With Bug1, the robot fully circles the object first, then departs from the point with the shortest distance toward the goal. This approach is, of course, very inefficient but it guarantees that the robot will reach any reachable goal.

- **Bug 2:**  With Bug2 the robot begins to follow the object’s contour, but departs immediately when it is able to move directly(on M-line) toward the goal. In general this improved Bug algorithm will have significantly shorter total robot travel. However, one can still construct situations in which Bug2 is arbitrarily inefficient.

You can see the behavior of simple Bug Algorithms here:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug-algorithms.jpg" height="200">	
</p>

## Implementation
We have implemented the three main bug algorithms in Python. Additionally, we simulated these algorithms on 3-wheel omnidirectional robot using [Webots](https://cyberbotics.com).

If your are just interested in controllers, there are three distinct Python files which bug algorithms have been coded there. The conrollers are in : `SBU_omni_robot/controllers/final_controller/`.

| Algorithm | Controller |
|--|--|
| *Bug 0* | [`BUG0_controller.py`](https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/SBU_omni_robot/controllers/final_controller/BUG0_controller.py) |
| *Bug 1* | [`BUG1_controller.py`](https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/SBU_omni_robot/controllers/final_controller/BUG1_controller.py) |
| *Bug 2* | [`BUG2_controller.py`](https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/SBU_omni_robot/controllers/final_controller/BUG2_controller.py) |


## Simulation
The simulation results in Webots are as follows. Also you can watch the full videos of the simulations here: [Simulation Videos](https://drive.google.com/drive/folders/13nMh-HdsPXXpsJykysohrLSxVexHf-t3?usp=sharing)
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/map.jpg" height="500">	
</p>

---
### Bug 0 snapshots:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug0_simulation.jpg" width="1000">	
</p>

---
### Bug 1 snapshots:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug1_simulation.jpg" width="1000">	
</p>

---
### Bug 2 snapshots:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug2_simulation.jpg" width="1000">	
</p>

## How to run
For simulation, you need to install [Webots](https://cyberbotics.com) software on your system. After installing Webots:
```
1. clone the repository
2. open /SBU_omni_robot project in Webots
3. Run and Happy Simulation!
``` 

## State diagrams
For better intuition, we also plot the state diagram of these three main bug algorithms:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug0_state_diagram.jpg" height="500">	
</p>
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug1_state_diagram.jpg" height="500">	
</p>
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug2_state_diagram.jpg" height="500">	
</p>

