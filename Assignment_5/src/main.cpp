// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5;       //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = false;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform) {
    /* 
    Constructing camera reference system given:
        1. eye position e
        2. gaze direction g
        3. view-up vector t
    */
    //TODO: setup uniform

    //TODO: setup camera, compute w, u, v
    const Vector3d w = (-camera_gaze).normalized(); // w = -g/||g||
    const Vector3d u = camera_top.cross(w).normalized(); // u = (t\times w)/(||t\times w||)
    const Vector3d v = w.cross(u); // v = w \times u

    //TODO: compute the camera transformation
    Matrix3d R; // rows are u^T, v^T, w^T
    R.row(0) = u.transpose();
    R.row(1) = v.transpose();
    R.row(2) = w.transpose();

    Matrix4d M_cam = Matrix4d::Identity();
    M_cam.block<3,3>(0,0) = R;
    M_cam.block<3,1>(0,3) = -R * camera_position; // translation part


    //TODO: setup projection matrix
    Matrix4d M_orth = Matrix4d::Identity();
    if (is_perspective)
    {
        //TODO setup prespective camera
    }
    else
    {
        // orthographic camera
        double n = near_plane;
        double f = far_plane;

        // Compute top (t) and right (r) from FOV, match near-plane size
        double t = std::tan(field_of_view / 2.0) * n;
        double r = aspect_ratio * t;
        double b = -t;
        double l = -r;

        M_orth(0,0) = 2.0 / (r - l);
        M_orth(1,1) = 2.0 / (t - b);
        M_orth(2,2) = -2.0 / (f - n);
        M_orth(0,3) = -(r + l) / (r - l);
        M_orth(1,3) = -(t + b) / (t - b);
        M_orth(2,3) = -(f + n) / (f - n);
    }
    uniform.view = M_cam;
    uniform.projective = M_orth;
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes vout = va;

        Eigen::Vector4d p_obj  = va.position;                    // (x,y,z,1)
        Eigen::Vector4d p_clip = uniform.projective * uniform.view * p_obj;   // clip
        const double w = p_clip[3];
        Eigen::Vector3d ndc = p_clip.head<3>() / w;               // NDC

        vout.position = Eigen::Vector4d(ndc.x(), ndc.y(), ndc.z(), 1.0);
        return vout;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        FragmentAttributes fa(1, 0, 0, 1);   // solid red
        fa.depth = va.position[2];           // NDC z in [-1,1] → definitely < 2
        return fa;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        FrameBufferAttributes out = previous;

        // If your framebuffer initializes depth to +inf, this keeps nearest
        if (fa.depth < previous.depth) {
            out.color[0] = fa.color[0];
            out.color[1] = fa.color[1];
            out.color[2] = fa.color[2];
            out.color[3] = fa.color[3];
            out.depth    = fa.depth;
        }
        return out;
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets
    vertex_attributes.reserve(facets.rows() * 3);
    for (int f = 0; f < facets.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const int idx = facets(f, k);
            vertex_attributes.emplace_back(
                vertices(idx,0), vertices(idx,1), vertices(idx,2), 1.0
            );
        }
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;

    // to rotate the object, we must first translate its barycenter to the origin, 
    // apply the rotation, and then translate it back to its original location.

    Eigen::Vector3d barycenter = vertices.colwise().mean();;

    Eigen::Matrix4d T_to_origin = Matrix4d::Identity(); // translates object to origin
    T_to_origin.block<3,1>(0,3) = -barycenter;

    Eigen::Matrix4d T_back = Matrix4d::Identity(); // translates object to original location
    T_back.block<3,1>(0,3) = barycenter;

    Eigen::Matrix4d R = Matrix4d::Identity(); // rotates object around z-axis
    R(0,0) = cos(alpha);  
    R(0,2) = sin(alpha);
    R(2,0) = -sin(alpha); 
    R(2,2) = cos(alpha);

    res = T_back * R * T_to_origin; // move to origin, apply rotation, move back.

    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);

    Matrix4d trafo = compute_rotation(alpha);
    Program program;

    program.VertexShader = [trafo](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes vout = va;

        Eigen::Vector4d p_obj  = va.position;                    // (x,y,z,1)
        Eigen::Vector4d p_world = trafo * p_obj;
        Eigen::Vector4d p_clip = uniform.projective * uniform.view * p_world;   // clip
        const double w = p_clip[3];
        Eigen::Vector3d ndc = p_clip.head<3>() / w;               // NDC

        vout.position = Eigen::Vector4d(ndc.x(), ndc.y(), ndc.z(), 1.0);
        return vout;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        FragmentAttributes fa(1, 0, 0, 1);   // solid red
        fa.depth = va.position[2];           // NDC z in [-1,1] → definitely < 2
        return fa;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        FrameBufferAttributes out = previous;

        // If your framebuffer initializes depth to +inf, this keeps nearest
        if (fa.depth < previous.depth) {
            out.color[0] = fa.color[0];
            out.color[1] = fa.color[1];
            out.color[2] = fa.color[2];
            out.color[3] = fa.color[3];
            out.depth    = fa.depth;
        }
        return out;
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    vertex_attributes.reserve(facets.rows() * 6); // 2 vertices per edge, 3 edges per triangle
    // Create edges from facets
    for (int f = 0; f < facets.rows(); ++f) {
        int i0 = facets(f, 0);
        int i1 = facets(f, 1);
        int i2 = facets(f, 2);

        // Edge 0-1
        vertex_attributes.emplace_back(vertices(i0,0), vertices(i0,1), vertices(i0,2), 1.0);
        vertex_attributes.emplace_back(vertices(i1,0), vertices(i1,1), vertices(i1,2), 1.0);

        // Edge 1-2
        vertex_attributes.emplace_back(vertices(i1,0), vertices(i1,1), vertices(i1,2), 1.0);
        vertex_attributes.emplace_back(vertices(i2,0), vertices(i2,1), vertices(i2,2), 1.0);

        // Edge 2-0
        vertex_attributes.emplace_back(vertices(i2,0), vertices(i2,1), vertices(i2,2), 1.0);
        vertex_attributes.emplace_back(vertices(i0,0), vertices(i0,1), vertices(i0,2), 1.0);
    }

    //TODO: use the transformation matrix
    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        //TODO: compute the correct lighting
        return va;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: compute the normals
    //TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);

    //TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes
    //TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    // simple_render(frameBuffer);
    // framebuffer_to_uint8(frameBuffer, image);
    // stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    //TODO: add the animation =======================================================
    const char* filename = "rotation.gif";
    GifWriter g;
    int delay = 25;
    GifBegin(&g, filename, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (double angle = 0; angle < 8 * M_PI; angle += 0.15)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(angle, frameBuffer);  // or flat_shading(angle, frameBuffer), etc.
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }

    GifEnd(&g);
    return 0;
}
