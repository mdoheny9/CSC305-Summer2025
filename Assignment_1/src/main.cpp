////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // TODO
    return u.x()*v.y() - u.y()*v.x();
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    // TODO
    Vector2d ab = b - a;
    Vector2d cd = d - c;
    Vector2d ac = c - a;

    double ab_cd = det(ab, cd);

    if (ab_cd == 0) { // segments are parallel, never intersect
        return false;
    } else {
        double t1 = det(ac, cd)/ab_cd;
        double t2 = -det(ab, ac)/ab_cd;
        return (0 <= t1 && t1 <= 1) && (0 <= t2 && t2 <= 1);
    }
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    double min_x = poly[0].x();
    double max_x = poly[0].x();
    double min_y = poly[0].y();
    double max_y = poly[0].y();
    int n = poly.size();

    for (int i = 0; i < n; i++) {
        min_x = std::min(min_x, poly[i].x());
        max_x = std::max(max_x, poly[i].x());
        min_y = std::min(min_y, poly[i].y());
        max_y = std::max(max_y, poly[i].y());
    }

    Vector2d outside(max_x + 1, query.y()); // guarenteed outside bounding box 

    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO
    int num_intersections = 0;

    for (int i = 0; i < n; i++) {
        Vector2d a = poly[i];
        Vector2d b = poly[(i+1) % n];
        
        if (intersect_segment(query, outside, a, b)) {
            num_intersections++;
        }
    }
    return (num_intersections % 2) == 1;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);
    // TODO
    double x, y, z;
    int num_points;

    in >> num_points; // first line is num_points
    while (in >> x >> y >> z) { 
        points.push_back(Vector2d(x, y));
    }

    in.close();
    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    // TODO
    std::ofstream out(filename);

    out << points.size() << std::endl;
    for (Vector2d point: points) {
        out << point.x() << " " << point.y() << " " << 0 << std::endl;
    }

    out.close();
    return;
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";

    std::vector<Vector2d> points = load_xyz(points_path);

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }
    save_xyz("output.xyz", result);

    return 0;
}
