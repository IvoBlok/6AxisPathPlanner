Project to both generate the paths and send the instructions for manufacturing arbitrary foam shapes with my KR125 KRC1 6 axis robotic arm. 

Since vulkan wasn't the focus, (the path planning and comms are), the base code for it is straight up ripped from vulkan-tutorial.com and extension upon this by the YT channel Mori TM. Of course further additions / modifications have been made to support the requirements I have for this project. You're ofcourse free to use this code in any noncommercial way you want, though crediting the sources I did is probably appropriate.
The mesh-intersect is mostly a copy of https://github.com/intents-software/mesh-plane-intersection/tree/master with modifications and additions made to both get it actually running and optimize it for my application.

Currently this project generates 'paths', in the sense that the location of the cutter is exported. No orientation or joint angles are exported/checked. Going forward I want to work on:

 - Fully support varying color along polylines
 - Fix transparency; Objects are only transparent relative to each other if their generation order is correct. Transparency also doesn't fully work perfect when you can see multiple surfaces of the same object through each other
 - Add non-planar pathing
 - Add robot + tool visualization
 - Add movement animation
 - Add inverse kinematics and collision control to validate paths

To keep note, one 'future works' idea might be to integrate collisions into the path planning; A given path potentially has 1+ extra degrees of freedom to play with in terms of orientation for a 6/7D robotarm. By defining some theoretical function that is maximized if a collision occurs, we might be able to apply a minimization problem to get the optimal values for the free degrees of freedom to avoid collision / exceeding the joint limits.

<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

Here's a blank template to get started. To avoid retyping too much info, do a search and replace with your text editor for the following: `github_username`, `repo_name`, `twitter_handle`, `linkedin_username`, `email_client`, `email`, `project_title`, `project_description`, `project_license`

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Dependencies

Though CMake should handle (most) dependencies, for the sake of completeness here are the external sources that are used. 

For the graphics:
* Vulkan
* glfw
* glm
* imgui
* implot

For file importing:
* stbImage
* tinyObjLoader

### Installation

1. Clone the repository.
   ```sh
   git clone https://github.com/IvoBlok/6AxisPathPlanner.git
   ```
2. Create a build directory.
   ```sh
   cd 6AxisPathPlanner
   mkdir build
   ```
3. Compile the path planner using your prefered compiler.
   ```sh
   cd build
   cmake ../
   cmake --build .
   ```
   The resulting executable is located in `build/Debug/`.
   
4. Compile the PCodeSender using g++ (on some linux variant).
   ```sh
   cd resources
   g++ -g -o PCodeSender PCodeSender.cpp
   ```
5. Transfer `KRLExternalControl.src` to the robot controller. Either by copying it line by line (safest), inserting some external media into the controller, or removing the primary harddrive and copying the file over in some external system (risky). The controller itself compiles the KRL code when needed. 


<!-- USAGE EXAMPLES -->
## Usage

This repository contains the path planner, which can be compiled using the instructions above. It also contains, within the resources folder, the code that is running on the Kuka robot controller and on the external computer sending the instructions. See the overview below for their relations. 

_For more examples, please refer to the [Documentation](https://example.com)_


<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/github_username/repo_name/issues) for a full list of proposed features (and known issues).


<!-- LICENSE -->
## License

You're ofcourse free to use this code in any noncommercial way you want, though crediting the sources I did is probably appropriate. Tadie License section!


<!-- CONTACT -->
## Contact

Ivo Blok - ivoblokdoorn@gmail.com

Project Link: [https://github.com/IvoBlok/6AxisPathPlanner](https://github.com/IvoBlok/6AxisPathPlanner)