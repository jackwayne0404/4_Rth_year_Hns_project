#pragma once
#include <stdint.h>
#include <GL/glew.h>
#include <cuda_runtime.h>
#define VISUALIZE
// Some constants
#define N          100000  // number of particles, i.e. boids + obstacles
#define NOBSTACLES 5     // number of obstacles
#define POS_STDDEV 0.5f    // initial position sigma
#define VEL_STDDEV 0.001f  // initial velocity sigma
#define DT         0.3f
#define RADIUS     1.0f    // maximum distance from the center the boids are allowed to move
#define VEL_MAX    0.01f   // maximum velocity
#define OBSTACLE_RADIUS 0.02f
#define OBSTACLE_REPULSION_RADIUS 0.1f
#define OBSTACLE_REPULSION_SCALE 0.001f
#define CENTER_ATTRACTION 0.0003f
#define MOUSE_ATTRACTION 0.0001f

// Parameters for the boids algorithm.
#define rule1Distance 0.05f
#define rule2Distance 0.03f
#define rule3Distance 0.05f

#define rule1Scale 0.01f
#define rule2Scale 0.05f
#define rule3Scale 0.02f

// particle flags
#define FLAGS_OBSTACLE 1
#define FLAGS_IN_REPULSION_RADIUS 2

// data stored in CUDA buffer
struct ParticleData {
	float3 pos;
	float3 vel;
	unsigned flags;
};

void Init(bool verbose
#ifdef VISUALIZE
	, GLuint boidbuffer // contains ParticleData
#endif
);
void SimulationStep();
void Mouse(float x, float y, float z, bool retract);
void NoMouse();
void Destroy();
