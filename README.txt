Coded by Edward Selig
This program was a project for my embedded systems class during the Fall 2015 semester.
It is meant to simulate a drone collision avoidance system by using multi-threading and mutexes.
I received an 100% on the project.
The program can be compiled by typing "make".  The program can be executed by typing
./DroneMap "Input File".  The valid input files are "level0.txt", "level1a.txt", 
"level1b.txt", "level2.txt".  In level 0 there is only one drone going back and return on
the same path.  In level 1a there are 10 drones going back and return on the same path,
unless a collision with another drone is detected. In level 1b and 2 you can see drones
avoiding collisions with "T"s (Trees) and other drones.

Assumptions:
For every "P" (Package) there is one drone.
The "S" (Station) is a static obstacle
