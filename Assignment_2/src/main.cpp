// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    const double sphere_radius = 0.9;
    const Vector3d sphere_center(0, 0, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            // TODO change this with the generic case
            double a = ray_direction.transpose() * ray_direction;
            double b = 2 * ray_direction.transpose() * (ray_origin - sphere_center);
            double c = (ray_origin - sphere_center).transpose() * (ray_origin - sphere_center) - sphere_radius * sphere_radius;
            
            double discr = b * b - 4 * a * c;

            if (discr >= 0) {
                double t = -1;
                double t0 = (-b - sqrt(discr)) / (2 * a);
                double t1 = (-b + sqrt(discr)) / (2 * a);
                if (t0 > 0) {
                    t = t0;
                } else if (t1 > 0) {
                    t = t1;
                }
                if (t > 0) {
                    Vector3d ray_intersection = ray_origin + t * ray_direction;
                    Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                    // Simple diffuse model
                    C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                    // Clamp to zero
                    C(i, j) = std::max(C(i, j), 0.);

                    // Disable the alpha mask for this pixel
                    A(i, j) = 1;
                }
            } 
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

bool intersect_parallelogram(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &pgram_origin, const Vector3d &pgram_u, const Vector3d &pgram_v, double &t, double &alpha, double &beta, Vector3d &ray_intersection) {
    // using ray-plane intersection approach
    Vector3d normal = (pgram_u.cross(pgram_v)).normalized();
    if (normal.dot(ray_direction) == 0) {
        return false; // ray is parallel to plane, no intersection
    }

    // solve for t in the rays parametric form: ray_origin + t * ray_direction = point on plane
    t = (pgram_origin - ray_origin).dot(normal) / normal.dot(ray_direction);
    if (t < 0) { 
        return false; // intersection is behind ray_origin, no intersection
    }

    ray_intersection = ray_origin + t * ray_direction;

    // ray intersects if 0 <= alpha, beta and alpha + beta <= 1
    Vector3d p = ray_intersection - pgram_origin;
    Vector3d n = pgram_u.cross(pgram_v);
    alpha = n.dot(p.cross(pgram_v)) / n.dot(n);
    beta  = n.dot(pgram_u.cross(p)) / n.dot(n);

    return (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1);
} 

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(1, 0.4, 0);
    const Vector3d pgram_v(0, 0.7, -10);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // TODO: Check if the ray intersects with the parallelogram
            Vector3d ray_intersection;
            double t, alpha, beta;
            if (intersect_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, t, alpha, beta, ray_intersection)) {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                // -> calculated within intersect_parallelogram

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (pgram_u.cross(pgram_v)).normalized(); 

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(1, 0.4, 0);
    const Vector3d pgram_v(0, 0.7, -10);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();

            // TODO: Check if the ray intersects with the parallelogram
            Vector3d ray_intersection;
            double t, alpha, beta;
            if (intersect_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, t, alpha, beta, ray_intersection)) {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                // -> calculated within intersect_parallelogram

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (pgram_u.cross(pgram_v)).normalized(); 

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

bool intersect_sphere(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &sphere_center, const double &sphere_radius, double &t, double &a, double &b, double &c, Vector3d &ray_intersection) {
    a = ray_direction.transpose() * ray_direction; // should be 1 if ray_direction normalized??? 
    b = 2 * ray_direction.transpose() * (ray_origin - sphere_center);
    c = (ray_origin - sphere_center).transpose() * (ray_origin - sphere_center) - sphere_radius * sphere_radius;
    
    double discr = b * b - 4 * a * c;
    if (discr < 0) {
        return false;
    }
    double t0 = (-b - sqrt(discr)) / (2 * a);
    double t1 = (-b + sqrt(discr)) / (2 * a);
    if (t0 > 0) {
        t = t0;
    } else if (t1 > 0) {
        t = t1;
    }
    if (t > 0) {
        ray_intersection = ray_origin + t * ray_direction;
        return true;
    } else {
        return false;
    }
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    // New matrices for RGB channels
    MatrixXd R = MatrixXd::Zero(800, 800);
    MatrixXd G = MatrixXd::Zero(800, 800);
    MatrixXd B = MatrixXd::Zero(800, 800);

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    const Vector3d light_intesity(1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();

            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection
            Vector3d ray_intersection;
            double t, a, b, c;
            if (intersect_sphere(ray_origin, ray_direction, sphere_center, sphere_radius, t, a, b, c, ray_intersection)) {
                // TODO: The ray hit the sphere, compute the exact intersection point
                // -> calculated within intersect_sphere

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // TODO: Add shading parameter here
                Vector3d l = (light_position - ray_intersection).normalized();
                Vector3d v = (camera_origin - ray_intersection).normalized();
                Vector3d n = ray_normal;
                Vector3d h = (v + l).normalized();

                double diffuse = std::max(0.0, n.dot(l)); // max(0, n · l)
                double specular = std::pow(std::max(0.0, n.dot(h)), specular_exponent); // max(0, n · h)^p

                // Simple diffuse model
                C(i, j) = ambient + diffuse + specular;

                // Using shading equation k_a I_a + k_d I max(0,n·l) + k_s I max(0, n·h)^p
                R(i, j) = ambient + diffuse_color[0] * diffuse + specular_color[0] * specular;
                G(i, j) = ambient + diffuse_color[1] * diffuse + specular_color[1] * specular;
                B(i, j) = ambient + diffuse_color[2] * diffuse + specular_color[2] * specular;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;

                
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
