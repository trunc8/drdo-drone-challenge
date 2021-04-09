# drdo-drone-challenge
Team:Bombay76

## Problem Statement

The task is to design an autonomous drone that navigates in a complex static environment by avoiding any collision with the on-field obstacles and reaching the target destination after its correct detection.

- Design an algorithm that autonomously navigates a drone from point A to point B avoiding collisions with the obstacles present in the path.
- The local coordinates of Point A would be known before-hand and the drone has to detect and navigate up to Point B ( that would be an Aruco Marker).
- The task is considered to be completed if the drone lands on Point B(Aruco Marker) without any crash. The ROS package must publish the ID of the Aruco marker and string “Landed” after landing on to topic “/aruco/message”
    - When not detected : “Marker ID : none, looking for marker”
    - When detected and landed : “ Marker ID : 0, Landed”
- Some parts of the Aruco marker must be visible to the RGB camera upon landing.
- Multiple Aruco Markers (false) may or may not be provided. The drone has to correctly identify the Aruco Marker based on the ID provided before landing. Correct Aruco ID will be ‘0’ in all the world.
- The drone model will be provided with a forward-facing depth camera and downward-facing RGB camera only (Any other sensors cannot be used).
- The flight should be strictly restricted to a height of 5m only.



## Steps to Run Locally
1. Follow all the [installation](installation/) steps
1. `cd ~/catkin_ws/src/`
1. `git clone https://github.com/Tech-Meet-Solutions/drdo-drone-challenge.git (https)`  
   OR  
   `git clone git@github.com:Tech-Meet-Solutions/drdo-drone-challenge.git (ssh)`
1. `catkin_make` OR `catkin build`
1. `cd drdo-drone-challenge/drdo_exploration`
1. `./start_sim.sh`

## Team
- Siddharth Saha- [trunc8](https://github.com/trunc8)
- Shubham Agrawal- [shubhamagr281999](https://github.com/shubhamagr281999)
- Pavan Kale- [pavanIITB](https://github.com/pavanIITB)
- Tejal Ashwini Barnwal- [tejalbarnwal](https://github.com/tejalbarnwal)
- Apoorva- [therealapoorva](https://github.com/therealapoorva)
- Shubham Gupta- [ShubhamGupta15](https://github.com/ShubhamGupta15)
- Ridayesh- [ridayesh](https://github.com/ridayesh)

