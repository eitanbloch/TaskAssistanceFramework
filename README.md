# TaskAssistanceFramework

# README

## Task Inspection Environment

We have an environment built to model two out of the three CLAIR lab robots 

URe3 and one of the URe5 

The files :

<details>
  <summary style="color:lightskyblue; font-size: 1.4em;">simulation</summary>
       contains all files related to the simulation of the robots

  <details>
  <summary style="color:lightblue;">building_blocks.py</summary>
 contains all the basic functions that are used in the simulation
such as local planning and collision checking
</details>

<details>
  <summary style="color:lightblue;">environment.py</summary>
    contains the environment that is used to model the obstacles
</details>

<details>
  <summary style="color:lightblue;">intervals_runner.py</summary>
     Implements path discretization, interval normalization, and visibility interval etc
</details>

<details>
  <summary style="color:lightblue;">inverse_kinematics.py</summary>
      contains the inverse kinematics functions and specifics for the URe robots 
</details>

<details>
  <summary style="color:lightblue;">kinematics.py</summary>
  contains the kinematics of the robot such as DH (denavit hartenberg) and geometric parameters 
</details>

<details>
  <summary style="color:lightblue;">RRG.py</summary>
    contains the RRG algorithm and the functions to build the tree
</details>

<details>
  <summary style="color:lightblue;">run.py</summary>
    presenting the usage of the main functions to run the simulation which behaves as an example of how to use this framework
</details>

<details>
  <summary style="color:lightblue;">visualizer.py</summary>
presents real-time visualization and plotting of the environmental 
obstacles and the robots using Matplotlib in 3D space
</details>

<br/>
</details>



<details>
  <summary style="color:lightskyblue; font-size: 1.4em;">URManipulators</summary>
    contains all files related to the real robots and camera control
<details>
  <summary style="color:lightblue;">CameraController.py</summary>
  contains the code that controls the OnRobot Eyes camera
</details>

<details>
  <summary style="color:lightblue;">execute_final_path.py</summary>
  contains the execution of the final path using the functions from Robots and can 
  serve as an example of how to use the real robots for path execution
</details>

<details>
  <summary style="color:lightblue;">generate_intervals.py</summary>
     compute the intervals during which an object is visible in the camera 
     feed while the task robot is executing its task used to generate nodes for graph
</details>

<details>
  <summary style="color:lightblue;">generate_samples.py</summary>
   generate samples for the RRG algorithm
</details>

<details>
  <summary style="color:lightblue;">Robots.py</summary>
   contains the basic needed functions to control the robots such as movej, movel, etc
</details>

</details>

### **Real Robots Connection Instructions**

How to connet to the robots?

### step 1 - connect to the TP-Link_8BC0 wifi

password is : 65729922

<img src="Misc/WifiName.png" alt="WifiName" width="300"/>


The router has no DHCP so the IP address needs to be configured manually 

![Untitled](Misc/ManualIP.png)

the IP adress should be in the range 192.168.0.20-192.168.0.30 and the mask should be 255.255.0.0.

Important IP addresses:

- URe5: 192.168.0.10
- URe3: 192.168.0.11
- Camera: 192.168.0.3

### Example Code

```jsx
from urx import Robot

ur5_ip = '192.168.0.10'
ur3_ip = '192.168.0.11'

ur5_robot = Robot(ur5_ip, use_rt=True)

# all joint angles are in rad

if __name__ == '__main__':
    home_conf = [0, -1.57, 0, -1.57, 0, 0]
    ur5_robot.movej(home_conf, acc=0.5, vel=0.3)
```




