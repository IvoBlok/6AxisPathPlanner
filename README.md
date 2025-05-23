Project to both generate the paths and send the instructions for manufacturing arbitrary foam shapes with my KR125 KRC1 6 axis robotic arm. 

Since vulkan wasn't the focus, (the path planning and comms are), the base code for it is straight up ripped from vulkan-tutorial.com and extension upon this by the YT channel Mori TM. Of course further additions / modifications have been made to support the requirements I have for this project. You're ofcourse free to use this code in any noncommercial way you want, though crediting the sources I did is probably appropriate.
The mesh-intersect is mostly a copy of https://github.com/intents-software/mesh-plane-intersection/tree/master with modifications and additions made to both get it actually running and optimize it for my application.

Currently this project generates 'paths', in the sense that the location of the cutter is exported. No orientation or joint angles are exported/checked. Going forward I want to work on:

 - Finalize the transition to CMake
 - Rewrite / Check the path planning code and general project structure as to prepare for future expansion
 - Add non-planar pathing
 - Add robot + tool visualization
 - Add movement animation
 - Add inverse kinematics and collision control to validate paths

To keep note, one 'future works' idea might be to integrate collisions into the path planning; A given path potentially has 1+ extra degrees of freedom to play with in terms of orientation. By defining some theoretical function that is maximized if a collision occurs, we might be able to apply a minimization problem to get the optimal values for the free degrees of freedom to avoid collision / exceeding the joint limits.