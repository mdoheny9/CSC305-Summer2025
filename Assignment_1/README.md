Assignment 1: Point-in-Polygon Algorithm
======================
## Overview
Implementation of 2D computational geometry algorithms in C++, focusing on the point-in-polygon test using the ray casting (even-odd rule) method.

## Algorithm Details
### Point-in-Polygon Implementation
The algorithm uses the ray casting method with these specific optimizations:

- Bounding Box Calculation: Computes min/max x,y coordinates of polygon vertices
- Exterior Point Selection: Uses `(max_x + 1, query.y())` to guarantee a point outside the polygon
- Ray Casting: Casts horizontal ray from query point to exterior point
- Intersection Counting: Tests ray against each polygon edge using parametric line intersection
- Even-Odd Rule: Odd intersection count = inside, even count = outside

### Line Segment Intersection Algorithm
Uses parametric representation and determinants:
```cpp
// For segments [a,b] and [c,d], solve: a + t1*(b-a) = c + t2*(d-c)
double t1 = det(ac, cd) / det(ab, cd);
double t2 = -det(ab, ac) / det(ab, cd);
// Intersection exists if 0 ≤ t1 ≤ 1 and 0 ≤ t2 ≤ 1
```

### Result

Here is an image of the dataset provided in this assignment:

<img width="876" height="653" alt="image" src="https://github.com/user-attachments/assets/f747aa0b-720c-4a8d-9a62-cbd5b5fadeeb" />


- The input `points.xyz` are shown in purple.
- The polygon in `polygon.obj` is shown in red.
- The points from `points.xyz` which are inside `polygon.obj` are shown in yellow.
