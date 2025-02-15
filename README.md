# TurtleBot3 Behavior Tree with py_trees

This repository contains a behavior tree implementation for TurtleBot3 using `py_trees`. The behavior tree is designed to control the robot's movement and avoid obstacles autonomously.

## Features

- Move Forward: The robot moves forward at a constant speed.

- Obstacle Detection: Uses LaserScan data to detect obstacles within 0.5m in front of the robot.

- Obstacle Avoidance: If an obstacle is detected, the robot stops and turns to avoid it.

- Behavior Tree Implementation: Uses py_trees to structure the robot's decision-making process.


## Future Work
- Add more complex behaviors (e.g., path planning, goal navigation).
- Improve obstacle detection using sensor fusion.
