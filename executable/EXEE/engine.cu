#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <memory>
#include <windows.h>
#include <algorithm> 
#include <vector> 

#include <GL/glew.h>
#include <thrust/sort.h>
#include <thrust/device_vector.h>
#include <device_launch_parameters.h>
#include <cuda_gl_interop.h>

#include "engine.h"

using namespace std;

// problem state

// dump detailed info
bool verbose;

// device buffer containing N positions
ParticleData *positions_d;

// /problem state

// these are initialized in Init()
float gridCellWidth;
int gridSideCount;
int gridCellCount;
float3 gridMinimum;

// thread block dimentions
int blockSize, blocks;


// CUDA stuff

template <typename T>
void check(T result, char const *const func, const char *const file,
	int const line) {
	if (result) {
		char msg[1000];
		sprintf(msg, "CUDA error at %s:%d code=%d(%s) \"%s\" \n", file, line,
			static_cast<unsigned int>(result), cudaGetErrorName(result), func);
		MessageBox(NULL, msg, "Error", MB_OK);
		//cudaDeviceReset();
		// Make sure we call CUDA Device Reset before exiting
		exit(EXIT_FAILURE);
	}
}

#define checkCudaErrors(val) check((val), #val, __FILE__, __LINE__)

// id of a thread
__device__ size_t globalId()
{
	return blockIdx.x * blockDim.x + threadIdx.x;
}

// number of threads
__device__ size_t globalSize()
{
	return gridDim.x * blockDim.x;
}

// Math utilities

__host__ __device__ int clamp(int x, int l, int u)
{
	if (x < l) return l;
	if (x >= u) return u - 1;
	return x;
}

__host__ __device__ float3 operator+(float3 a, float3 b)
{
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__host__ __device__ void operator+=(float3 &a, float3 b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
}
__host__ __device__ float3 operator-(float3 a, float3 b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
__host__ __device__ void operator-=(float3 &a, float3 b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
}
__host__ __device__ float3 operator*(float3 a, float b)
{
	return make_float3(a.x * b, a.y * b, a.z * b);
}
__host__ __device__ float3 operator*(float b, float3 a)
{
	return make_float3(b * a.x, b * a.y, b * a.z);
}
__host__ __device__ void operator*=(float3 &a, float b)
{
	a.x *= b;
	a.y *= b;
	a.z *= b;
}

__host__ __device__ float distance3(float3 a, float3 b)
{
	a -= b;
	return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}

__host__ __device__ float length3(float3 a)
{
	return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}

__host__ __device__ float dot(float4 a, float3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

__host__ __device__ float dot(float3 a, float3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

__host__ __device__ float sqr(float x)
{
	return x*x;
}

// Boid simulation

// Method of computing a 1D index from a 3D grid index.
__host__ __device__ int gridIndex3Dto1D(int x, int y, int z, int gridResolution)
{
	return x + y * gridResolution + z * gridResolution * gridResolution;
}

__global__ void computeIndices(int gridResolution,
	float3 gridMin, float inverseCellWidth,
	ParticleData * __restrict__ pos, int * __restrict__ indices, int * __restrict__ gridIndices)
{
	int index = globalId();
	if (index >= N) {
		return;
	}

	// Go through the boids and determine which grid cell to bin them into
	float3 boidPos = (pos[index].pos - gridMin) * inverseCellWidth;
	int x = clamp((int)boidPos.x, 0, gridResolution);
	int y = clamp((int)boidPos.y, 0, gridResolution);
	int z = clamp((int)boidPos.z, 0, gridResolution);
	gridIndices[index] = gridIndex3Dto1D(x, y, z, gridResolution);
	// Set up a parallel array of integer indices as pointers to the actual
	// boid data in pos and vel1/vel2
	indices[index] = index;
}

// This is useful for indicating that a cell does not enclose any boids
// Called at the beginning of every step of a simulation to reset the buffer values to a default value which 
// tells us if the cell holds any boids or not
__global__ void resetIntBuffer(int length, int *intBuffer, int value)
{
	int index = globalId();
	if (index < length) {
		intBuffer[index] = value;
	}
}

// Identify the start and end points of each gridcell in the gridIndices array.
__global__ void identifyCellStartEnd(
	const int * __restrict__ particleGridIndices,
	int * __restrict__ gridCellStartIndices, 
	int * __restrict__ gridCellEndIndices)
{
	//go through particleGridIndices identifying when there is a change in there value, 
	//which signifies a change in the gridcell we are dealing with
	int index = globalId();
	if (index >= N) {
		return;
	}

	if (index == 0) { //edge case
		gridCellStartIndices[particleGridIndices[index]] = 0;
	}
	else if (index == N - 1) { //edge case
		gridCellEndIndices[particleGridIndices[index]] = N - 1;
	}
	else if (particleGridIndices[index] != particleGridIndices[index + 1]) {
		//inbetween grid cells with no boids are set to -1  --> done before when both the arrays were reset to -1

		//change in gridcell
		gridCellEndIndices[particleGridIndices[index]] = index;
		gridCellStartIndices[particleGridIndices[index + 1]] = index + 1;
	}
}

// Make the particle array more memory coherent
__global__ void permuteParticles(
	const int * __restrict__ permutation,
	const ParticleData * __restrict__ pos, 
	ParticleData * __restrict__ sorted)
{
	int index = globalId();
	if (index >= N) {
		return;
	}

	sorted[index] = pos[permutation[index]];
}

__global__ void updateVelocities(
	int gridResolution, float3 gridMin,
	float inverseCellWidth, float cellWidth,
	const int * __restrict__ gridCellStartIndices, 
	const int * __restrict__ gridCellEndIndices,
	const ParticleData *__restrict__ input,
	ParticleData *__restrict__ output,
	float3 mouse,
	bool mousePressed, bool mouseRetract)
{
	// gridCellStartIndices and gridCellEndIndices refer to coherentPos and coherentVel.

	int index = globalId();
	if (index >= N) {
		return;
	}
	ParticleData particle = input[index];

	if (particle.flags & FLAGS_OBSTACLE) {
		particle.vel = make_float3(0, 0, 0);
		output[index] = particle;
		return; // obstacles don't move
	}

	// find boid position
	// then use that position to determine the gridcell it belongs to
	// use that information to find the 27 cells you have to check
	float3 boidPos = (particle.pos - gridMin) * inverseCellWidth;
	int x = boidPos.x;
	int y = boidPos.y;
	int z = boidPos.z;

	float3 v1 = { 0.0f, 0.0f, 0.0f };
	float3 v2 = { 0.0f, 0.0f, 0.0f };
	float3 v3 = { 0.0f, 0.0f, 0.0f };

	float3 percieved_center_of_mass = { 0.0f, 0.0f, 0.0f };
	float3 perceived_velocity = { 0.0f, 0.0f, 0.0f };
	float3 separate_vector = { 0.0f, 0.0f, 0.0f };
	float3 repulsion = { 0.0f, 0.0f, 0.0f };

	int neighborCount1 = 0;
	int neighborCount3 = 0;

	float distance = 0.0f;
	float3 newVel = particle.vel;

	particle.flags &= ~FLAGS_IN_REPULSION_RADIUS; // clear this flag

	// For best results, consider what order the cells should be
	// checked in to maximize the memory benefits of reordering the boids data.
	for (int dz = -1; dz <= 1; dz++) {
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				int _x = clamp(x + dx, 0, gridResolution);
				int _y = clamp(y + dy, 0, gridResolution);
				int _z = clamp(z + dz, 0, gridResolution);

				int boidGridCellindex = gridIndex3Dto1D(_x, _y, _z, gridResolution);

				// Identify which cells may contain neighbors. This isn't always 8.
				// For each cell, read the start/end indices in the boid pointer array.

				if (gridCellStartIndices[boidGridCellindex] != -1) {
					//we know the grid cell is empty if its start or end indices have been set to -1

					//now go through the boids in that grid cell and apply the rules 
					//to it if it falls within the neighbor hood distance
					for (int h = gridCellStartIndices[boidGridCellindex]; h <= gridCellEndIndices[boidGridCellindex]; h++) {
						if (h != index) {
							ParticleData another = input[h];
							// Access each boid in the cell and compute velocity change from
							// the boids rules, if this boid is within the neighborhood distance.
							float3 delta = another.pos - particle.pos;
							distance = length3(delta);
							if (another.flags & FLAGS_OBSTACLE) {
								// rules for boid-obstacle interaction
								if (distance < OBSTACLE_REPULSION_RADIUS) {
									// 1e-5 here to supress division by zero
									repulsion -= delta * (OBSTACLE_REPULSION_SCALE / (distance + 1e-5));
									particle.flags |= FLAGS_IN_REPULSION_RADIUS;
								}
							}
							else {
								// rules for boid-boid interaction
								if (distance < rule1Distance) {
									percieved_center_of_mass += another.pos;
									neighborCount1++;
								}

								if (distance < rule2Distance) {
									separate_vector -= delta;
								}

								if (distance < rule3Distance) {
									perceived_velocity += another.vel;
									neighborCount3++;
								}
							}
						}
					}
				}
			}
		}
	}

	// Rule 1: boids fly towards their local perceived center of mass, which excludes themselves
	if (neighborCount1 != 0) {
		percieved_center_of_mass *= 1.0f / neighborCount1;
		v1 = (percieved_center_of_mass - particle.pos) * rule1Scale;
	}

	// Rule 2: boids try to stay a distance rule2Distance away from each other
	v2 = separate_vector * rule2Scale;

	// Rule 3: boids try to match the speed of surrounding boids
	if (neighborCount3 != 0) {
		perceived_velocity *= 1.0 / neighborCount3;
		v3 = perceived_velocity * rule3Scale;
	}

	newVel += v1 + v2 + v3;

	// Obstacle repulsion
	newVel += repulsion;

	// Try to stay within a certain radius
	float fromCenter = dot(particle.pos, particle.pos);
	if (fromCenter > RADIUS*RADIUS) {
		newVel = newVel - particle.pos * (CENTER_ATTRACTION / fromCenter);
	}

	if (mousePressed) {
		// attract to the mouse pointer
		float3 delta = mouse - particle.pos;
		float d2 = sqr(delta.x) + sqr(delta.y) + sqr(delta.z);
		if (d2 > 1e-6) {
			float3 force = delta * (MOUSE_ATTRACTION / sqr(d2));
			if (mouseRetract) newVel -= force;
			else newVel += force;
		}
	}

	// Clamp the speed change before putting the new speed in vel2
	float absVel = length3(newVel);
	if (absVel > VEL_MAX) {
		newVel = newVel * (VEL_MAX / length3(newVel));
	}
	particle.vel = newVel;

	particle.pos += particle.vel * DT; // update position

	output[index] = particle;
}


// reordered particles
ParticleData *sortedParticles_d;

int *arrayIndices_d;
int *gridIndices_d;

int *cellStartIndices_d;
int *cellEndIndices_d;

#ifdef VISUALIZE
// GL buffer
GLuint glBuffer;
cudaGraphicsResource_t glResource;

void ensurePoints_d()
{
	checkCudaErrors(cudaGraphicsMapResources(1, &glResource));
	size_t size;
	checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void**)&positions_d, &size, glResource));
	if (size != sizeof(ParticleData) * N) {
		fprintf(stderr, "Unexpected GL points buffer size: %ld (expected %d * %d = %ld)\n", size, (int)sizeof(float3), N, sizeof(float3) * N);
		exit(-1);
	}
}

void doneWithPoints_d()
{
	checkCudaErrors(cudaGraphicsUnmapResources(1, &glResource));
}
#endif

void Init(bool _verbose
#ifdef VISUALIZE
	, GLuint _glBuffer
#endif
)
{
	verbose = _verbose;

	blockSize = 128;
	blocks = (N + blockSize - 1) / blockSize;

#ifdef VISUALIZE
	glBuffer = _glBuffer;
	unsigned int cudaDeviceCount;
	int cudaDevice;
	checkCudaErrors(cudaGLGetDevices(&cudaDeviceCount, &cudaDevice, 1, cudaGLDeviceListAll));
	if (cudaDeviceCount == 0) {
		fprintf(stderr, "No CUDA devices correspond to the current GL context\n");
		exit(-1);
	}
	if (verbose) printf("Choose CUDA device %d\n", cudaDevice);
	checkCudaErrors(cudaSetDevice(cudaDevice));

	checkCudaErrors(cudaGraphicsGLRegisterBuffer(&glResource, glBuffer, cudaGraphicsRegisterFlagsNone));

	ensurePoints_d();
#else
	checkCudaErrors(cudaSetDevice(0));
	checkCudaErrors(cudaMalloc(&positions_d, sizeof(ParticleData) * N));
#endif

	// set to normally distributed positions and velocities
	ranlux24_base rng;
	rng.seed(random_device()());
	normal_distribution<float> posDist(0.0f, POS_STDDEV);
	normal_distribution<float> obstacleRadDist(RADIUS * 0.9f, 0.1f * RADIUS);
	normal_distribution<float> velDist(0.0f, VEL_STDDEV);

	auto positions = make_unique<ParticleData[]>(N);

	// generate obstacles
	int i;
	for (i = 0; i < NOBSTACLES; i++) {
		// Generate an obstacle near the RADIUS sphere
		float3 pos = { posDist(rng), posDist(rng), posDist(rng) };
		pos = pos * (obstacleRadDist(rng) / (length3(pos) + 1e-3));
		positions[i] = { pos, make_float3(0, 0, 0), FLAGS_OBSTACLE };
	}

	// generate boids
	for (; i < N; i++) {
		positions[i] = { make_float3(posDist(rng), posDist(rng), posDist(rng)),
			make_float3(velDist(rng), velDist(rng), velDist(rng)), 0 };
	}

	// upload random positions/velocities data to GPU
	checkCudaErrors(cudaMemcpy(positions_d, positions.get(), sizeof(ParticleData) * N, cudaMemcpyHostToDevice));

	gridCellWidth = max(max(max(rule1Distance, rule2Distance), rule3Distance), OBSTACLE_REPULSION_RADIUS);
	gridSideCount = ceil(RADIUS * 2 / gridCellWidth);
	gridCellCount = gridSideCount * gridSideCount * gridSideCount;
	gridMinimum = make_float3(-RADIUS, -RADIUS, -RADIUS);

	if (verbose) {
		printf("gridCellWidth: %f\ngridSideCount: %d\ngridCellCount: %d\n", gridCellWidth, gridSideCount, gridCellCount);
	}

	checkCudaErrors(cudaMalloc(&arrayIndices_d, sizeof(int) * N));
	checkCudaErrors(cudaMalloc(&gridIndices_d, sizeof(int) * N));
	checkCudaErrors(cudaMalloc(&sortedParticles_d, sizeof(ParticleData) * N));
	checkCudaErrors(cudaMalloc(&cellStartIndices_d, sizeof(int) * gridCellCount));
	checkCudaErrors(cudaMalloc(&cellEndIndices_d, sizeof(int) * gridCellCount));

#ifdef VISUALIZE
	doneWithPoints_d();
#endif
}

void Destroy()
{
#ifndef VISUALIZE
	cudaFree(positions_d);
#endif

	cudaFree(arrayIndices_d);
	cudaFree(gridIndices_d);
	cudaFree(sortedParticles_d);
	cudaFree(cellStartIndices_d);
	cudaFree(cellEndIndices_d);

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cudaDeviceReset();
}

float3 mouse = { 0, 0, 0 };
bool mousePressed = false;
bool retract = false;

// mouse press/move
void Mouse(float x, float y, float z, bool _retract)
{
	mouse = make_float3(x, y, z);
	mousePressed = true;
	retract = _retract;
}

// mouse release
void NoMouse()
{
	mousePressed = false;
}

void SimulationStep()
{
#ifdef VISUALIZE
	// sync with OpenGL
	ensurePoints_d();
#endif

	// Uniform Grid Neighbor search using Thrust sort with cell-coherent data.

	int gridBlocks = (gridCellCount + blockSize - 1) / blockSize;

	// Reset buffers start and end indices buffers
	resetIntBuffer <<<gridBlocks, blockSize >>> (gridCellCount, cellStartIndices_d, -1);
	resetIntBuffer <<<gridBlocks, blockSize >>> (gridCellCount, cellEndIndices_d, -1);

	cudaDeviceSynchronize();

	// Label each particle with its array index as well as its grid index.
	// Use 2x width grids
	// recompute grid cell indices and particlearray indices every timestep
	computeIndices <<<blocks, blockSize >>> (gridSideCount, 
		gridMinimum, 1.0f / gridCellWidth,
		positions_d, arrayIndices_d,
		gridIndices_d);

	cudaDeviceSynchronize();
	
	// Now sort the boids so that boids belonging to the same grid cell
	// are next to each other .
	// Wrap device vectors in thrust iterators for use with thrust.
	thrust::device_ptr<int> dev_thrust_keys(gridIndices_d);
	thrust::device_ptr<int> dev_thrust_values(arrayIndices_d);
	// Unstable key sort using Thrust. A stable sort isn't necessary
	thrust::sort_by_key(dev_thrust_keys, dev_thrust_keys + N, dev_thrust_values);

	identifyCellStartEnd <<<blocks, blockSize >>> (gridIndices_d, cellStartIndices_d, cellEndIndices_d);

	// use the rearranged array index buffer to reshuffle all the boid data(position, velocity and obstacle flag)
	// in the simulation array, such that it is memory coherent arranged in order of grid cells
	permuteParticles <<<blocks, blockSize >>> (arrayIndices_d, positions_d, sortedParticles_d);
	
	// Perform velocity updates using neighbor search
	updateVelocities <<<blocks, blockSize >>> (gridSideCount, gridMinimum,
		1.0f / gridCellWidth, gridCellWidth,
		cellStartIndices_d, cellEndIndices_d,
		sortedParticles_d, positions_d, 
		mouse, mousePressed, retract);

#ifdef VISUALIZE
	// sync with OpenGL
	doneWithPoints_d();
#endif
}

