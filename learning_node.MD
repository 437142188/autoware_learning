# autoware学习笔记

autoware整体架构如下：
 ![image](https://github.com/437142188/autoware_learning/blob/learning_note/picture/Screenshot%20from%202018-06-27%2017-06-32.png)

## 3-D Map Generation and Sharing
 The most notable feature of Autoware is the fact that the autonomous driving of Autoware
uses a 3-D map. The 3-D map, unlike 2-D maps used in car navigation system, is a map
including various information, such as roads and all the still objects around the roads. The
3-D map will play a critical role in deploying autonomous driving into the real world.
Autoware localizes the position of the ego-vehicle by matching the 3-D map loaded into the
PC with the surrounding information captured by the LIDAR on the roof. The precision of
localization is approximately 10cm, which number indicates far higher precision than GPS.
 
## Localization (NDT:Normal Distributions Transform)
 The position of the ego-vehicle can be located by scan matching based on the NDT
algorithm with the 3-D map of PCD (Point Cloud Data) format and LIDAR data. The position
error of localization is around 10cm.

## Object Detection
DPM (Deformable Part Models) algorithms can detect cars, pedestrians and traffic lights
from camera images. Tracking using Kalman filter can be implemented and it improves
detection accuracy. Fusing 3-D LIDAR data can obtain the distances of the detected objects.
Traffic lights detection is conducted as following steps. First, the 3-D positions of traffic lights
are calculated by using the localization result and the high-definition 3-D map. Next, the 3-D
positions of traffic lights are projected on camera images by sensor fusion. Then, traffic light
colors are recognized by image processing.

## Path Generation
AutowareRoute (a route data generation application using MapFan) generates a path to the
selected destination. Then, autonomous driving system determines the lanes on the path.
The path data generation application is provided as a car navigation of smartphone. The
trajectories in the lanes are calculated kinematically.

## Autonomous Driving
The generated path includes appropriate speed information. Autonomous driving system
uses it as the target speed. In addition, the route includes landmarks, “way point”, set at 1m
intervals. The autonomous driving system operates “path following” by following the way
points. Referring near way points on curves and distant way points on straights stabilizes
autonomous driving. If ego-vehicle deviates from the route, the system aims to the vicinity
point and back to the target route. Note that the safest path is selected by the driving system
