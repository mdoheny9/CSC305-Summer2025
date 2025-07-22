////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

private:
    // builds the bvh recursively
    int build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles);
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

// Camera settings
const double focal_length = 10;
const double field_of_view = 0.7854; // 45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 5);

//Maximum number of recursive calls
const int max_bounce = 5;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
// Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    // setup tree
    bvh = AABBTree(vertices, facets);

     //Spheres
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);

    //parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // Vector containing the list of tringle indices
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    root = build_recursive(V, F, centroids, 0, triangles.size(), -1, triangles);
}

int AABBTree::build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles)
{
    // Scene is empty, so is the aabb tree
    if (to - from == 0)
    {
        return -1;
    }

    // If there is only 1 triangle left, then we are at a leaf
    if (to - from == 1)
    {
        // TODO create leaf node and retun correct left index

        return -1;
    }

    // TODO sort centroids along the longest dimension

    // TODO Use AlignedBox3d to find the box around the current centroids
    AlignedBox3d centroid_box;

    // Diagonal of the box
    Vector3d extent = centroid_box.diagonal();

    // TODO find the largest dimension
    int longest_dim = 0;

    std::sort(triangles.begin() + from, triangles.begin() + to, [&](int f1, int f2)
              {
        //TODO sort the **triangles** along the centroid largest dimension
        // return true if triangle f1 comes before triangle f2
        return false; });

    // TODO Create a new internal node and do a recursive call to build the left and right part of the tree
    // TODO finally return the correct index

    return -1;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

//Compute the intersection between a ray and a sphere, return -1 if no intersection
double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // DONE, implement the intersection between the ray and the sphere at index index.
    //return t or -1 if no intersection

    const Vector3d sphere_center = sphere_centers[index];
    const double sphere_radius = sphere_radii[index];

    double t = -1;

    double a = ray_direction.transpose() * ray_direction;
    double b = 2 * ray_direction.transpose() * (ray_origin - sphere_center);
    double c = (ray_origin - sphere_center).transpose() * (ray_origin - sphere_center) - sphere_radius * sphere_radius;
    
    double discr = b * b - 4 * a * c;

    if (discr < 0)
    {
        return -1;
    }
    else
    {
        //DONE set the correct intersection point, update p to the correct value
        double t0 = (-b - sqrt(discr)) / (2 * a);
        double t1 = (-b + sqrt(discr)) / (2 * a);
        if (t0 > 0) {
            t = t0;
        } else if (t1 > 0) {
            t = t1;
        }
        p = ray_origin + t * ray_direction;
        N = (p - sphere_center).normalized();

        return t;
    }
}

//Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // DONE, implement the intersection between the ray and the parallelogram at index index.
    //return t or -1 if no intersection

    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;

    Vector3d normal = (pgram_u.cross(pgram_v)).normalized();
    if (normal.dot(ray_direction) == 0)
    {
        return -1;
    }

    double t = (pgram_origin - ray_origin).dot(normal) / normal.dot(ray_direction);
    if (t < 0) { 
        return -1; // intersection is behind ray_origin, no intersection
    }

    //DONE set the correct intersection point, update p and N to the correct values
    Vector3d ray_intersection = ray_origin + t * ray_direction;

    Vector3d p_rel = ray_intersection - pgram_origin;
    Vector3d n = pgram_u.cross(pgram_v);
    double alpha = n.dot(p_rel.cross(pgram_v)) / n.dot(n);
    double beta  = n.dot(pgram_u.cross(p_rel)) / n.dot(n);

    if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1) {
        p = ray_intersection;
        N = -normal;
        return t;
    } else {
        return -1;
    }
}

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    Vector3d pgram_u = b - a;
    Vector3d pgram_v = c - a;
    Vector3d normal = (pgram_u.cross(pgram_v)).normalized();

    Vector3d rhs = a - ray_origin;
    Matrix3d M;
    M.col(0) = ray_direction;
    M.col(1) = -pgram_u;
    M.col(2) = -pgram_v;

    if (M.determinant() == 0)
    {
        return -1;
    }

    Vector3d lhs = M.inverse() * rhs;
    double t = lhs.x();
    double u = lhs.y();
    double v = lhs.z();

    if (u < 0 or v < 0 or u + v > 1)
    {
        return -1;
    }
    p = ray_origin + t * ray_direction;
    N = normal;
    return t;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    double txmin, txmax, tymin, tymax, tzmin, tzmax;
    txmin = (box.min().x() - ray_origin.x()) / ray_direction.x();
    txmax = (box.max().x() - ray_origin.x()) / ray_direction.x();
    if (txmin > txmax) std::swap(txmin, txmax);

    tymin = (box.min().y() - ray_origin.y()) / ray_direction.y();
    tymax = (box.max().y() - ray_origin.y()) / ray_direction.y();
    if (tymin > tymax) std::swap(tymin, tymax);

    if ((txmin > tymax) || (tymin > txmax)) {
        return false;
    }
    if (tymin> txmin) {
        txmin = tymin;
    }
    if (tymax < txmax) {
        txmax = tymax;
    }

    tzmin = (box.min().z() - ray_origin.z()) / ray_direction.z();
    tzmax = (box.max().z() - ray_origin.z()) / ray_direction.z();
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((txmin > tzmax) || (tzmin > txmax)) {
        return false;
    }

    if (tzmin > txmin) {
        txmin = tzmin;
    }
    if (tzmax < txmax) {
        txmax = tzmax;
    }
    return true;
}

// Finds the closest intersecting object returns its index
// In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"
    bool is_hit = false;

    // TODO
    // Triangle Mesh Method (1): Traverse every triangle and return the closest hit.
    for (int i = 0; i < facets.rows(); i++) {
        Vector3d a = vertices.row(facets(i, 0));
        Vector3d b = vertices.row(facets(i, 1)); 
        Vector3d c = vertices.row(facets(i, 2));
        const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
        if (t >= 0) {
            if (t < closest_t) {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                is_hit = true;
            }
        }
    }

    // Sphere
    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0) {
            if (t < closest_t) {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;  
                is_hit = true;
            }
        }
    }

    // Parallelogram
    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0) {
            if (t < closest_t) {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                is_hit = true;
            }
        }
    }
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    return is_hit;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

//Checks if the light is visible
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &light_position)
{
    // TODO: Determine if the light is visible here
    // Use find_nearest_object
    Vector3d obj_p, obj_n;
    const double e = 1e-4;
    if (find_nearest_object(ray_origin + e * ray_direction, ray_direction, obj_p, obj_n)) {
        const double dlight = (light_position - ray_origin).norm();
        const double dhit = (obj_p - (ray_origin + e * ray_direction)).norm();
        return dhit > dlight;
    }
    return true;
    
}

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce)
{
    // Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i) {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        const Vector3d Li = (light_position - p).normalized();

        // TODO: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible
        if (is_light_visible(p, Li, light_position)) {
            Vector4d diff_color = obj_diffuse_color;

            if (nearest_object) {
                //Compute UV coodinates for the point on the /* sphere */
                const double x = p(0) - sphere_centers[nearest_object][0];
                const double y = p(1) - sphere_centers[nearest_object][1];
                const double z = p(2) - sphere_centers[nearest_object][2];
                double tu = acos(z / sphere_radii[nearest_object]) / 3.1415;
                double tv = (3.1415 + atan2(y, x)) / (2 * 3.1415);
                tu = std::min(tu, 1.0);
                tu = std::max(tu, 0.0);

                tv = std::min(tv, 1.0);
                tv = std::max(tv, 0.0);

                // diff_color = procedural_texture(tu, tv);
            }
            // Diffuse contribution
            const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);
            // Specular contribution
            const Vector3d Hi = (Li - ray_direction).normalized();
            const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
            // Attenuate lights according to the squared distance to the lights
            const Vector3d D = light_position - p;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }
    }

    Vector4d refl_color = obj_reflection_color;
    if (nearest_object) {
        refl_color = Vector4d(0.5, 0.5, 0.5, 0);
    }
    // TODO: Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color
    Vector4d reflection_color(0, 0, 0, 0);
    if (max_bounce > 0) {
        Vector3d v = -ray_direction.normalized(); // ray_direction points towards the surface so sign must change
        Vector3d r = 2 * N * (N.dot(v)) - v; // direction of reflected ray
        const double e = 1e-4; 
        reflection_color = refl_color.cwiseProduct(shoot_ray(p + e * r, r, max_bounce - 1));
    }

    // TODO: Compute the color of the refracted ray and add its contribution to the current point color.
    //       Make sure to check for total internal reflection before shooting a new ray.
    Vector4d refraction_color(0, 0, 0, 0);

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color + refraction_color;

    // Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    // TODO
    double image_y = tan(field_of_view / 2) * focal_length;
    double image_x = aspect_ratio * image_y;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}