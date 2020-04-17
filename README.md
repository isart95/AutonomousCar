# AutonomousCar

**Simulation of an autonomous car in IST (Lisboa)**

Project developed in Python - ROS - (Gazebo/RViz)

**Provisional Architecture:**

  1.  Supervisor (Guidance, Navigation)
    a.  Path planning & Trajectory generation modul:
    c.  Event handling
    d.  Energy management and monitoring (Energy budget)

        -Initial & final configuration
        -Set of intermediate configurations to guide the path through specific regions
        -Set of locations where specific events will happen
        -Returns trajectory that will be "fed to the low level control"


  2.  Motion control
    a.  Model of the car and the kinematics
        -Allow the car to move between any two arbitrary configurations (initial and final pose)
        -Only points in traffic roads are admissible

  3.  Localization (¿? -> Ask teacher)
    a.  Apply Gaussian error to wheel sensors

  4.  Graphical interface ()


-   Inputs:

    1.  Pose of the car (x,y,theta)
    2.  Linear velocity of the car (v at time = t)
    3.  Angular position of the steering wheel (phi at time = t)

-   Outputs:
    1.  Final pose of the car (x,y,theta) at final time (t + diff(t))

*Car knows its configuration within a pre-specified error.*

==============================================================================
**Outcome:**
Zip/rar file containing all the software developed, tests ran, and adequate pdf
documentation explaining, clearly, the project and how to use the software.
===============================================================================

**Authors**:

- Pethros Tzathas
- Veronica Spelbrink
- Isart Julià

==========
**Tasks**:
- [] Set meeting with teacher for Monday 20/04
- [] Meeting on 20/04 - *Clearfy architechture*
- []
