// Include standard headers

/*
http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/ // model loader
https://www.3dgep.com/opengl-interoperability-with-cuda/  // excellent CUDA OPEnGl interlop example
https://www.nvidia.com/content/GTC/documents/1055_GTC09.pdf // 
https://gamedevelopment.tutsplus.com/tutorials/3-simple-rules-of-flocking-behaviors-alignment-cohesion-and-separation--gamedev-3444
*/



#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <random>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
using namespace glm;
using namespace std;

#include "shader.h"
#include "controls.h"
#include "objloader.h"
#include "engine.h"
#include "speedsample.h"

void createObstacles(std::vector<glm::vec3> positions, std::vector<glm::vec3> &vertices, std::vector<glm::vec3> &normals)
{
	std::vector<glm::vec3> svertices;
	std::vector<glm::vec3> snormals;
	if (!loadOBJ("sphere.obj", svertices, snormals)) exit(-1);

	vertices.reserve(svertices.size() * positions.size()); // preallocate memory
	normals.reserve(svertices.size() * positions.size()); // preallocate memory

	for (auto it = positions.begin(); it != positions.end(); it++) {
		glm::vec3 pos = *it;

		// translate & scale the shere
		for (int i = 0; i < svertices.size(); i++) {
			vertices.push_back(svertices[i] * OBSTACLE_RADIUS + pos);
		}
		normals.insert(normals.end(), snormals.begin(), snormals.end());
	}
}

int main(void)
{
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 1);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Boid Flocking", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	glfwPollEvents();

	// Black background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint obstacleProgramID = LoadShaders("sphere.vertexshader", "sphere.fragmentshader");
	GLuint boidProgramID = LoadShaders("boid.vertexshader", "boid.fragmentshader");

	// Load it into a VBO

	GLuint boidbuffer;
	glGenBuffers(1, &boidbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, boidbuffer);
	glBufferData(GL_ARRAY_BUFFER, N * sizeof(ParticleData), NULL, GL_STATIC_DRAW);

	// Init the boid engine; this will generate random boids and obstacles in the same buffer
	// the first NO. of oBSTACLE elements in the buffer are obstacles
	Init(true, boidbuffer);

	// read obstacle positions
	vector<ParticleData> obstacleParticles(NOBSTACLES);
	glBindBuffer(GL_ARRAY_BUFFER, boidbuffer);
	glGetBufferSubData(GL_ARRAY_BUFFER, 0, NOBSTACLES * sizeof(ParticleData), obstacleParticles.data());

	vector<glm::vec3> obstaclePositions;
	for (int i = 0; i < NOBSTACLES; i++) {
		float3 v = obstacleParticles[i].pos;
		obstaclePositions.push_back({ v.x, v.y, v.z });
	}
	vector<glm::vec3> vertices;
	vector<glm::vec3> normals;
	createObstacles(obstaclePositions, vertices, normals);

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint normalbuffer;
	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

	SpeedSample ss(20);
	int i = 0;

	do {

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Compute the MVP matrix from keyboard and mouse input
		computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();
		glm::mat4 ModelMatrix = glm::mat4(1.0);
		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
		glm::vec3 lightPos(4.0f, 4.0f, 4.0f); // in camera coordinates

		// Draw obstacles

		glUseProgram(obstacleProgramID);

		// initialize uniforms

		glUniform3f(glGetUniformLocation(obstacleProgramID, "LightPosition_cameraspace"),
			lightPos.x, lightPos.y, lightPos.z);
		glUniformMatrix4fv(glGetUniformLocation(obstacleProgramID, "MVP"), 
			1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(glGetUniformLocation(obstacleProgramID, "V"), 
			1, GL_FALSE, &ViewMatrix[0][0]);
		glUniformMatrix4fv(glGetUniformLocation(obstacleProgramID, "M"),
			1, GL_FALSE, &ModelMatrix[0][0]);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : normals
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
		glVertexAttribPointer(
			1,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the triangles !
		glDrawArrays(GL_TRIANGLES, 0, vertices.size());

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

		// Draw boids

		glUseProgram(boidProgramID);

		// initialize uniforms

		glUniform3f(glGetUniformLocation(boidProgramID, "LightPosition_cameraspace"),
			lightPos.x, lightPos.y, lightPos.z);
		glUniformMatrix4fv(glGetUniformLocation(boidProgramID, "MVP"),
			1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(glGetUniformLocation(boidProgramID, "V"),
			1, GL_FALSE, &ViewMatrix[0][0]);
		glUniformMatrix4fv(glGetUniformLocation(boidProgramID, "M"),
			1, GL_FALSE, &ModelMatrix[0][0]);

		// boid positions
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, boidbuffer);
		glVertexAttribPointer(
			0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			sizeof(ParticleData), // stride
			(void*)0            // offset: first field of ParticleData
		);

		// boid flags
		glEnableVertexAttribArray(1);
		glVertexAttribIPointer(
			1,                  // attribute
			1,                  // size
			GL_UNSIGNED_INT,    // type
			sizeof(ParticleData),// stride
			(void*)24            // offset: third field of ParticleData
		);

		// Draw the points
		glPointSize(1.2f);
		glDrawArrays(GL_POINTS, 0, N);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

		// Swap buffers
		glfwSwapBuffers(window);
		
		SimulationStep();

		glfwPollEvents();

		if (++i % 20 == 0) {
			ss.sample(20);
			printf(" %f FPS        \r", ss.getSpeed());
		}


	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteProgram(obstacleProgramID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

