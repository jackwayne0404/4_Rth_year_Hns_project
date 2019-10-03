#ifndef CONTROLS_HPP
#define CONTROLS_HPP
#include <GL/glew.h>
#include <glm.hpp>

#define WIN_WIDTH 1000
#define WIN_HEIGHT 1000

void computeMatricesFromInputs();
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();

#endif