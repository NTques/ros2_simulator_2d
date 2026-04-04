#pragma once
#include <string>
#include <vector>
#include <utility>

struct RobotState {
    double x      = 0.0;   // world position (m)
    double y      = 0.0;
    double theta  = 0.0;   // heading (rad), CCW from +x

    double cmd_vx     = 0.0;  // linear  velocity from /cmd_vel
    double cmd_vtheta = 0.0;  // angular velocity from /cmd_vel

    double radius = 0.2;   // circumscribed radius — auto-computed from footprint if set

    // Polygon footprint in robot-local frame (empty → fall back to circle collision)
    std::vector<std::pair<double,double>> footprint;
};

// Parse URDF and return the collision radius of the base link.
// Falls back to 0.2 m if parsing fails.
double parseURDFRadius(const std::string& urdf_path);

// Parse nav2-style footprint string:  "[ [x1,y1], [x2,y2], ... ]"
// Returns empty vector on parse failure.
std::vector<std::pair<double,double>> parseFootprintString(const std::string& s);

// Circumscribed radius of a polygon (max distance from origin to any vertex).
double footprintRadius(const std::vector<std::pair<double,double>>& poly);
