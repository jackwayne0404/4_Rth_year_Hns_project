#include "controls.h"
// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window; 
#include <iostream>

// Include GLM
#include <gtc/matrix_transform.hpp>
using namespace glm;

#include "engine.h"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

bool fullScreen = false;
bool f11pressed = false;
int lastXpos, lastYpos, lastWidth, lastHeight; // saved window size

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}


// Initial position : on +Z
glm::vec3 position = glm::vec3( 0, 0, 3.0f );
glm::vec3 up = glm::vec3(0, 1, 0);

// Initial Field of View
float initialFoV = 45.0f;
float aspect = 1.0f;

bool callbacksSet = false;

void scrollCallback(GLFWwindow*, double dx, double dy)
{
	// Move forward/backward
	position *= pow(0.97f, dy);
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
	aspect = (float)width / height;
	glViewport(0, 0, width, height);
}

void computeMatricesFromInputs()
{
	if (!callbacksSet) {
		glfwSetScrollCallback(window, scrollCallback);
		callbacksSet = true;
		glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	}

	// Get mouse position
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	bool leftButton = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	bool rightButton = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
	if (leftButton || rightButton) {
		int ww, wh;
		glfwGetFramebufferSize(window, &ww, &wh);
		xpos = (xpos - ww/2) / (ww/2);
		ypos = (wh / 2 - ypos) / (wh / 2);
		// transform mouse pointer to model coordinates using iterative algorithm
		float dist = glm::length(position);
		glm::vec3 camera(0, 0, RADIUS - dist);
		auto invView = glm::inverse(ViewMatrix);
		for (int i = 0; i < 10; i++) {
			float w = camera.z * ProjectionMatrix[2][3]; // column-major!
			camera.x = xpos * w / ProjectionMatrix[0][0];
			camera.y = ypos * w / ProjectionMatrix[1][1];

			if (camera.x*camera.x + camera.y*camera.y < RADIUS*RADIUS) {
				camera.z += dist;
				camera = camera * (RADIUS / glm::length(camera)); // make the point lie on the sphere
				camera.z -= dist;
			}
		}
		glm::vec4 p = invView * glm::vec4(camera, 1);
		Mouse(p.x, p.y, p.z, rightButton); // right button retracts, left button attracts
	}
	else {
		NoMouse();
	}

	glm::vec3 right = glm::cross(up, position);

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	// Move forward
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		position = glm::vec3(glm::rotate(glm::mat4(), 0.05f, right) * glm::vec4(position, 1.0f));
	}
	// Move backward
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		position = glm::vec3(glm::rotate(glm::mat4(), -0.05f, right) * glm::vec4(position, 1.0f));
	}
	// Strafe right
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		position = glm::vec3(glm::rotate(glm::mat4(), -0.05f, up) * glm::vec4(position, 1.0f));
	}
	// Strafe left
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		position = glm::vec3(glm::rotate(glm::mat4(), 0.05f, up) * glm::vec4(position, 1.0f));
	}
	if (glfwGetKey(window, GLFW_KEY_F11) == GLFW_PRESS) {
		if (!f11pressed) {
			f11pressed = true;
			// switch full screen
			if (fullScreen) {
				fullScreen = false;
				glfwSetWindowMonitor(window, NULL, lastXpos, lastYpos, lastWidth, lastHeight, GLFW_DONT_CARE);
			}
			else {
				fullScreen = true;
				glfwGetWindowPos(window, &lastXpos, &lastYpos);
				glfwGetWindowSize(window, &lastWidth, &lastHeight);
				const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
				if (mode) {
					glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, GLFW_DONT_CARE);
				}
			}
		}
	}
	else {
		f11pressed = false;
	}
	up -= position * glm::dot(up, position) / glm::dot(position, position);
	up /= glm::length(up); // normalize

	float FoV = initialFoV;
	
	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), aspect, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix       = glm::lookAt(
								position,           // Camera is here
								glm::vec3(0,0,0), // and looks here : at the same position, plus "direction"
								up                  
						   );

}