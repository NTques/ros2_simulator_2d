#include "simulator/robot.hpp"
#include <urdf/model.h>
#include <cmath>
#include <cctype>
#include <iostream>
#include <cstdio>
#include <stdexcept>
#include <string>

// Run `xacro <path>` and return the resulting URDF XML string.
// Throws std::runtime_error on failure.
static std::string runXacro(const std::string& xacro_path) {
    // Quote the path to handle spaces
    std::string cmd = "xacro \"" + xacro_path + "\" 2>&1";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("popen failed for xacro command");
    }
    std::string result;
    char buf[4096];
    while (fgets(buf, sizeof(buf), pipe)) {
        result += buf;
    }
    int rc = pclose(pipe);
    if (rc != 0) {
        // Include first 200 chars of output as diagnostic
        throw std::runtime_error(
            "xacro failed (exit code " + std::to_string(rc) + "):\n" +
            result.substr(0, 200));
    }
    return result;
}

static bool endsWithIgnoreCase(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    std::string tail = s.substr(s.size() - suffix.size());
    for (auto& c : tail) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return tail == suffix;
}

// ---------------------------------------------------------------------------
// Footprint helpers
// ---------------------------------------------------------------------------
std::vector<std::pair<double,double>> parseFootprintString(const std::string& s) {
    std::vector<std::pair<double,double>> result;
    const char* p   = s.c_str();
    const char* end = p + s.size();

    while (p < end) {
        // Advance to next '['
        while (p < end && *p != '[') ++p;
        if (p >= end) break;
        ++p;  // consume '['

        // Skip whitespace
        while (p < end && std::isspace(static_cast<unsigned char>(*p))) ++p;

        // If next char is '[' this is the outer wrapper bracket — skip it
        if (p >= end || *p == '[') continue;

        // Parse x
        char* nend = nullptr;
        double x = std::strtod(p, &nend);
        if (!nend || nend == p) { ++p; continue; }
        p = nend;

        // Skip to ','
        while (p < end && *p != ',') ++p;
        if (p >= end) break;
        ++p;  // consume ','

        // Skip whitespace
        while (p < end && std::isspace(static_cast<unsigned char>(*p))) ++p;

        // Parse y
        double y = std::strtod(p, &nend);
        if (!nend || nend == p) { ++p; continue; }
        p = nend;

        result.emplace_back(x, y);
    }
    return result;
}

double footprintRadius(const std::vector<std::pair<double,double>>& poly) {
    double r2 = 0.0;
    for (const auto& [x, y] : poly)
        r2 = std::max(r2, x * x + y * y);
    return std::sqrt(r2);
}

// ---------------------------------------------------------------------------
double parseURDFRadius(const std::string& urdf_path) {
    urdf::Model model;
    bool success = false;

    if (endsWithIgnoreCase(urdf_path, ".xacro")) {
        std::cout << "[simulator] Processing xacro: " << urdf_path << "\n";
        try {
            std::string xml = runXacro(urdf_path);
            success = model.initString(xml);
        } catch (const std::exception& e) {
            std::cerr << "[simulator] xacro error: " << e.what()
                      << " — using default radius 0.2 m\n";
            return 0.2;
        }
    } else {
        success = model.initFile(urdf_path);
    }

    if (!success) {
        std::cerr << "[simulator] Failed to parse URDF/xacro: " << urdf_path
                  << " — using default radius 0.2 m\n";
        return 0.2;
    }

    // Candidate base link names (ordered by preference)
    for (const auto& link_name : {"base_footprint", "base_link", "chassis"}) {
        auto link = model.getLink(link_name);
        if (!link) continue;

        // Prefer collision geometry; fall back to visual
        std::shared_ptr<urdf::Geometry> geom;
        if (link->collision && link->collision->geometry)
            geom = link->collision->geometry;
        else if (link->visual && link->visual->geometry)
            geom = link->visual->geometry;

        if (!geom) continue;

        switch (geom->type) {
            case urdf::Geometry::CYLINDER: {
                auto cyl = std::static_pointer_cast<urdf::Cylinder>(geom);
                std::cout << "[simulator] Robot radius from URDF cylinder: "
                          << cyl->radius << " m\n";
                return cyl->radius;
            }
            case urdf::Geometry::SPHERE: {
                auto sph = std::static_pointer_cast<urdf::Sphere>(geom);
                std::cout << "[simulator] Robot radius from URDF sphere: "
                          << sph->radius << " m\n";
                return sph->radius;
            }
            case urdf::Geometry::BOX: {
                auto box = std::static_pointer_cast<urdf::Box>(geom);
                // Use half-diagonal of footprint as conservative radius
                double r = std::hypot(box->dim.x, box->dim.y) / 2.0;
                std::cout << "[simulator] Robot radius from URDF box: " << r << " m\n";
                return r;
            }
            default:
                break;
        }
    }

    std::cerr << "[simulator] No recognised base-link geometry in URDF — "
                 "using default radius 0.2 m\n";
    return 0.2;
}
