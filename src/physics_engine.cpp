#include "simulator/physics_engine.hpp"
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
bool PhysicsEngine::circleVsCircle(double cx1, double cy1, double r1,
                                    double cx2, double cy2, double r2) const {
    double dx = cx2 - cx1, dy = cy2 - cy1;
    double sum_r = r1 + r2;
    return dx * dx + dy * dy < sum_r * sum_r;
}

// Circle vs possibly-rotated rectangle (centre rx,ry; half-extents rw/2, rh/2; angle ra)
bool PhysicsEngine::circleVsRect(double cx, double cy, double r,
                                  double rx, double ry,
                                  double rw, double rh,
                                  double ra) const {
    // Transform circle centre into rectangle's local frame
    double cos_a =  std::cos(-ra);
    double sin_a =  std::sin(-ra);
    double lx =  (cx - rx) * cos_a - (cy - ry) * sin_a;
    double ly =  (cx - rx) * sin_a + (cy - ry) * cos_a;

    double hw = rw * 0.5, hh = rh * 0.5;
    double nearest_x = std::clamp(lx, -hw, hw);
    double nearest_y = std::clamp(ly, -hh, hh);

    double dx = lx - nearest_x, dy = ly - nearest_y;
    return dx * dx + dy * dy < r * r;
}

// ---------------------------------------------------------------------------
bool PhysicsEngine::isFree(double x, double y, double radius,
                            const MapData&               map,
                            const std::vector<Obstacle>& obstacles) const {
    if (map.isValid() && map.isCircleOccupied(x, y, radius)) return false;

    for (const auto& obs : obstacles) {
        if (obs.shape == ObstacleShape::CIRCLE) {
            if (circleVsCircle(x, y, radius, obs.x, obs.y, obs.radius))
                return false;
        } else {
            if (circleVsRect(x, y, radius,
                             obs.x, obs.y, obs.width, obs.height, obs.angle))
                return false;
        }
    }
    return true;
}

// Sliding collision response:
//   1. Try full move
//   2. Try x-only
//   3. Try y-only
//   4. Stay put
void PhysicsEngine::move(double& rx, double& ry,
                          double  nx, double  ny,
                          double  radius,
                          const MapData&               map,
                          const std::vector<Obstacle>& obstacles) const {
    if (isFree(nx, ny, radius, map, obstacles)) {
        rx = nx; ry = ny;
        return;
    }
    if (isFree(nx, ry, radius, map, obstacles)) {
        rx = nx;
        return;
    }
    if (isFree(rx, ny, radius, map, obstacles)) {
        ry = ny;
        return;
    }
    // Blocked entirely — do not move
}

// ---------------------------------------------------------------------------
// Polygon footprint overloads
// ---------------------------------------------------------------------------

std::vector<std::pair<double,double>>
PhysicsEngine::transformFootprint(double x, double y, double theta,
                                   const std::vector<std::pair<double,double>>& local_fp)
{
    std::vector<std::pair<double,double>> world_fp;
    world_fp.reserve(local_fp.size());
    const double c = std::cos(theta), s = std::sin(theta);
    for (const auto& [lx, ly] : local_fp)
        world_fp.emplace_back(x + c*lx - s*ly,
                               y + s*lx + c*ly);
    return world_fp;
}

bool PhysicsEngine::isFree(double x, double y, double theta,
                            const std::vector<std::pair<double,double>>& footprint,
                            double circum_r,
                            const MapData&               map,
                            const std::vector<Obstacle>& obstacles) const
{
    // Map check — accurate polygon vs occupied cells
    if (map.isValid()) {
        auto world_fp = transformFootprint(x, y, theta, footprint);
        if (map.isPolygonOccupied(world_fp)) return false;
    }

    // Obstacle check — use circumscribed circle (conservative, fast)
    for (const auto& obs : obstacles) {
        if (obs.shape == ObstacleShape::CIRCLE) {
            if (circleVsCircle(x, y, circum_r, obs.x, obs.y, obs.radius))
                return false;
        } else {
            if (circleVsRect(x, y, circum_r,
                             obs.x, obs.y, obs.width, obs.height, obs.angle))
                return false;
        }
    }
    return true;
}

void PhysicsEngine::move(double& rx, double& ry, double theta,
                          double  nx, double  ny,
                          const std::vector<std::pair<double,double>>& footprint,
                          double circum_r,
                          const MapData&               map,
                          const std::vector<Obstacle>& obstacles) const
{
    if (isFree(nx, ny, theta, footprint, circum_r, map, obstacles)) {
        rx = nx; ry = ny;
        return;
    }
    if (isFree(nx, ry, theta, footprint, circum_r, map, obstacles)) {
        rx = nx;
        return;
    }
    if (isFree(rx, ny, theta, footprint, circum_r, map, obstacles)) {
        ry = ny;
        return;
    }
    // Blocked entirely — do not move
}
