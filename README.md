# Bug-Algorithms-Simulation

This is the final project of the *Fundamental of Robotic* course at Shahid Beheshti University.

## Introduction to Bug algorithms
The Bug algorithm is perhaps the simplest obstacle-avoidance algorithm one could imagine. The basic idea is to follow the contour of each obstacle in the robot’s way and thus circumnavigate it. A number of variations and extensions of the Bug algorithm exist, otherwise we have implemented the three main ones in Python.

- **Bug 0:** With Bug0, the robot begins to follow the object’s contour, but departs immediately when it is able to move directly toward the goal. In general this algorithm does not guarantees that the robot reach the goal.

- **Bug 1:**  With Bug1, the robot fully circles the object first, then departs from the point with the shortest distance toward the goal. This approach is, of course, very inefficient but it guarantees that the robot will reach any reachable goal.

- **Bug 2:**  With Bug2 the robot begins to follow the object’s contour, but departs immediately when it is able to move directly(on M-line) toward the goal. In general this improved Bug algorithm will have significantly shorter total robot travel. However, one can still construct situations in which Bug2 is arbitrarily inefficient.

You can see the behavior of simple Bug Algorithms here:
<p align="center">
  <img src="https://github.com/mohammadhashemii/Bug-Algorithms-Simulation/blob/main/images/bug-algorithms.jpg" width="200" height="200">	
</p>