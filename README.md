# CSC305-Summer2025

All assignments for **CSC305: Introduction to Computer Graphics** at UVic, Summer 2025.

## Overview

* Assignment 1: Point-in-polygon testing and geometric algorithms
* Assignment 2: Ray tracing with sphere/parallelogram intersection
* Assignment 3: Advanced ray tracing (shadows, reflections, Perlin noise)
* Assignment 4: BVH acceleration structures
* Assignment 5: OpenGL rasterization with shaders

See individual assignment READMEs for program output images and detailed descriptions.

## Setup

1. Download the assignment code
2. Create a directory called `build` in the assignment directory, e.g.:
   ```
   cd assignment_X; mkdir build
   ```
3. Use CMake to generate the Makefile/project files needed for compilation inside the `build/` directory:
   ```
   cd build; cmake -DCMAKE_BUILD_TYPE=Release ..
   ```
4. Compile and run the compiled executable by typing:
   ```
   make; ./assignmentX
   ```
