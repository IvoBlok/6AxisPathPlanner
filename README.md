Project to both generate the paths and send the instructions for manufacturing arbitrary foam shapes with my KR125 KRC1 6 axis robotic arm. 
Vulkan rendering used to show a preview of what the paths will be like for debugging.

Since vulkan wasn't the focus, (the path planning and comms are), the base code for it is straight up ripped from vulkan-tutorial.com and extension upon this by the YT channel Mori TM. Of course extensions have been made to support the requirements I have for this project. You're ofcourse free to use this code in any noncommercial way you want, though crediting the sources I did is probably appropriate.
The Slicer.h is mostly a copy of https://github.com/intents-software/mesh-plane-intersection/tree/master with modifications and additions made to both get it actually running and optimize it for my application.

personal TODOs for this project:
 - Find a fix for the edgecase for OffsetCurve where: 
	* there are two line segments after each other in the input curve
	* there is a decently small angle between these two segments, and the two segments are concave
	* the untrimmed offsets of these two segments are far enough away
   This results in the untrimmed offset of one being too close to the input polyline, but it doesn't intersect with the other offset, since it is far away. The connecting lines/arcs that fill the gaps won't collide, making no self-intersections, thus it won't be removed for being too close to the input curve.
   The same can happen theoretically happen with a line segment in between two arcs, but the web example shows that apparently the gap filling chooses the opposite point, forcing there to be a self-intersect.

 - Add simplification/smoothing of polygonal curves, see this page: https://www.cosy.sbg.ac.at/~held/projects/apx/apx.html for their solution, which seems very applicable, though ofcourse also complicated to implement

 - Add a milling path mode where it follows the surface orthogonally, so the resulting surface can be semi-smooth instead of stepwise