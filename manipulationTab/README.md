manipulationTab
===============
A tab to test manipulation techniques. Currently, a basic Jacobian-based grasping algorithm has been implemented with the latest
HUBOPLUS model.


`manipulationTab.cpp`
---------------------
Initializes Grasper according to HUBO model. In addition to previous tabs, you can now test open and closing hand strategies. Memory management checks have been included to allow for continuous testing without having to re-open program.

`Grasper.cpp`
-------------
Please note that this is just a basic manipulation strategy in order to show DART's ability to cope with manipulation tasks in a dynamic environment. Basic functionality is to use a Jacobian to move the arm's end effector towards calculated grasping point in the target object. Once there, the hand is closed until finger's distal links collide with the target object. Once the object is grasped, the robot tries to move it to 3 different locations within reach in the simulation environment.
	
`JointMover.cpp`
----------------
This code is a modification of original JTFollower.cpp provided by Ana Huaman where a translation Jacobian is computed for a robot's end effector. Additions include several speedups, configurations comparison, among others.

----------------
For step-by-step instructions, refer to [[Manipulation Tab Tutorial]]
