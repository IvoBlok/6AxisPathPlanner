#ifndef DATASTRUCTS_HPP
#define DATASTRUCTS_HPP

#define GLM_FORCE_SWIZZLE
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLMF_FORCE_DEFAULT_ALIGNED_GENTYPES
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/hash.hpp>

#include <vector>
#include <utility>

#include "cavc/vector2.hpp"
#include "cavc/vector3.hpp"
#include "cavc/polyline.hpp"

class DesiredShape {
public:
	std::vector<cavc::Vector3<double>> vertices;
	std::vector<cavc::Vector3<double>> normals;
	std::vector<uint32_t> indices;

	// TODO fix applyTransformation so that it uses either a custom mat4, or define the interaction between a glm::mat4 and customVector
	/*
	void applyTransformation(glm::mat4 transformation) {
		// update the vertices
		for (size_t i = 0; i < vertices.size(); i++)
		{
			vertices[i] = transformation * glm::vec4{ vertices[i], 1.f };
		}

		// update the normals
		for (size_t i = 0; i < normals.size(); i++)
		{
			normals[i] = transformation * glm::vec4{ normals[i], 0.f };
			normals[i] = glm::normalize(normals[i]);
		}
	}*/
};

// this stuct describes all the info relevant to this 
struct ToolInfo {
	// this is the radius of the main, cylinder shaped part of the tool in millimeters
	double mainToolRadius;
	// this is the total height from the tip of the cutting edges up to the end of the cutting edges in millimeters. This defines how deep of a maximum cut can be taken at once.
	double toolCuttingHeight;
	// this describes the amount of flutes on the tool
	int fluteCount;

	// this defines if the tool has a flat end, or circular end. Any tool that has a tool end different is currently not supported. Thus the radius of the ballend is the same as mainToolRadius
	bool toolIsBallEnd;
};

// this struct describes all info related to the piece of material which is to be milled into shape
struct StockInfo {
	// this describes the zero point from which the 'rectangular' stock extends into the three positive axes; i.e. it is the point with the lowest x, y, z coordinate that still lays within the stock
	// this point is relatively arbitrary, since it is in relation to an arbitrary simulated 'world' zero point, but is still relevant since the robot location is also defined with respect to this 'world' zero.
	// this point is defined in meters
	cavc::Vector3<double> zeroPoint;
	// this describes the height of the stock (z-axis) in millimeters
	double height;
	// this describes the width of the stock (x-axis) in millimeters
	double width;
	// this describes the length of the stock (y-axis) in millimeters
	double length;

	// material properties? 
};

// this struct describes milling robot related variables, like its relative location, axes lengths etc
// currently assumed is the style of robotic arm used, i.e. those used on most industrial mid-size kuka robots like the one for my application, the KR125/2
// currently assumed is that the robot is placed flat to the ground, with the ground also being coplanar with the top and bottom surfaces of the stock
struct RobotInfo {
	// this describes the position of the robot, where specifically this point is in line with the rotation vector of axis 1 and at the height of the real world floor.
	cavc::Vector3<double> zeroPoint;

	// this describes the default home position of the attachment end point (TCP) relative to the zero point of the robot. The orientation is not regarded as necessary to be given for now, since the home point should be far
	cavc::Vector3<double> homePoint;

	// for each axis, the axis rotation vector describes the direction around which it rotates relative to the previous axis
	// for each axis, the axis translation describes the vector from the zero point of the previous axis to the zero point of the next, where the zero point of an axis lies on the line defined by the axis rotation and is on the touching plane of the two axes
	cavc::Vector3<double> axisRotation[6];
	cavc::Vector3<double> axisTranslation[6];

	// for the arbitrary attachment *kuch* attached to the end of the robot, these variables define the translation and rotations defining the transformation from the axis 6 end point to the tool position and orientation
	cavc::Vector3<double> robotAttachmentTranslation;
	cavc::Vector3<double> robotAttachmentRotations;
};

// this struct describes the info required to generate and verify a single 2.5D toolpass
struct MillingPass2_5DInfo {
	// this describes the shape that we want to be milled
	DesiredShape desiredShape;

	// this describes the robot related variables like relative location axis lengths and attachment location. 
	// this is used for generation of the paths, but more importantly:
	// - verifying that the generated paths are free of collisions between the stock and the robot/ attachment
	// - verifying that the parts of the robot/attachment don't collide with itself
	// - verifying that the whole path within this pass is within reach of the robot
	RobotInfo robotInfo;

	// this describes the tool used within this milling pass
	ToolInfo toolInfo;

	// this describes the stock that is milled during this pass
	StockInfo stockInfo;

	// this describes the distance between paths within the same 2.5D plane in millimeters. This should be smaller then twice the toolInfo 'mainToolRadius'.
	double stepOver;

	// this desribes the distance between 2.5D planes in millimeters. This should be smaller then the toolInfo 'toolCuttingHeight'.
	double depthOfCut;

	// this describes the height in the direction of the plane normal, starting from the heighest 2.5D layer. In millimeters
	double safeTraverseHeight;
};

double PI = 3.141592653;
double TAU = PI * 2.f;

#endif