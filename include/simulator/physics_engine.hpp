#pragma once
#include "map_loader.hpp"
#include "obstacle_manager.hpp"
#include <vector>
#include <utility>

class PhysicsEngine {
public:
    // ---- Circle-based collision (legacy / fallback) ----

    bool isFree(double x, double y, double radius,
                const MapData&               map,
                const std::vector<Obstacle>& obstacles) const;

    void move(double& rx, double& ry,
              double  nx, double  ny,
              double  radius,
              const MapData&               map,
              const std::vector<Obstacle>& obstacles) const;

    // ---- Polygon-footprint collision ----
    // footprint: polygon vertices in robot-local frame
    // theta:     robot heading (used to transform footprint to world frame)
    // circum_r:  circumscribed radius used for fast obstacle-vs-circle check
    //            (conservative but avoids expensive polygon-polygon intersection)

    bool isFree(double x, double y, double theta,
                const std::vector<std::pair<double,double>>& footprint,
                double circum_r,
                const MapData&               map,
                const std::vector<Obstacle>& obstacles) const;

    void move(double& rx, double& ry, double theta,
              double  nx, double  ny,
              const std::vector<std::pair<double,double>>& footprint,
              double circum_r,
              const MapData&               map,
              const std::vector<Obstacle>& obstacles) const;

private:
    bool circleVsCircle(double cx1, double cy1, double r1,
                        double cx2, double cy2, double r2) const;

    bool circleVsRect(double cx, double cy, double r,
                      double rx, double ry,
                      double rw, double rh,
                      double ra) const;

    // Transform local-frame polygon to world frame
    static std::vector<std::pair<double,double>>
    transformFootprint(double x, double y, double theta,
                       const std::vector<std::pair<double,double>>& local_fp);
};
