# Autonomous Mini Rover
A rover with Raspberry Pi, Ultrasonic sensors, L293D motor driver and a battery that can detect and avoid the obstacles on its way and reach its destination.

Developed Path 2 Path Reachability using physics of a Two Wheeled Robot in TwoWheeledRobot.m. The Two Wheeled Robot follows the Dubin's path in descretized space.

Path from source to destination is achieved using Q Learning with predefined obstacles in 2D God's eye space in q_learning.m.

### Example in 50 x 50 space with origin (9, 14) and destination (27, 31) 

###### Path predicted with Q Learning 

![Path using Q Learning](https://github.com/vijaykvganaraju/autocar/blob/master/screenshots/Q_learn.jpg)



###### Dubin's path with Two Wheeled Robot Physics

![Dubin's Path with Two Wheeled Robot](https://github.com/vijaykvganaraju/autocar/blob/master/screenshots/Robot%20path.jpg)



To run the code, download all the file and run "main.m".

**rows** is the number of rows in the discretized space.
 
 **cols** is the number of colums in the discretized space.
 
 **initial** \[x, y\] is the origin point in 2D space.
 
 **target** \[x, y\] is the origin point in 2D space.
 
 **obstacles** is a vector of 2D points which define the obstacles where the Two Wheeled Robot cannot maneuver. 
 
 **angle** is the angle, the Two Wheeled Robot is facing, in 2D space.
 
