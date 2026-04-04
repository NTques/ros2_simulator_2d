#pragma once
#include <vector>
#include <cmath>

enum class ObstacleShape { CIRCLE, RECTANGLE };

struct Obstacle {
    int           id      = 0;
    ObstacleShape shape   = ObstacleShape::CIRCLE;

    // Current world position (m) & orientation (rad, CCW)
    double x     = 0.0;
    double y     = 0.0;
    double angle = 0.0;

    // Shape parameters
    double radius = 0.3;    // CIRCLE only
    double width  = 0.5;    // RECTANGLE: length along local x
    double height = 0.5;    // RECTANGLE: length along local y

    // Dynamic properties
    bool   dynamic  = false;
    double start_x  = 0.0;
    double start_y  = 0.0;
    double end_x    = 0.0;
    double end_y    = 0.0;
    double speed    = 0.5;  // m/s
    int    dir      = 1;    // +1: moving toward end, -1: moving toward start
};

class ObstacleManager {
public:
    // Returns the newly assigned id
    int  addObstacle(Obstacle obs);
    void removeObstacle(int id);
    void updateObstacle(const Obstacle& obs);
    void updateDynamic(double dt);
    void clear();

    std::vector<Obstacle>  getAll()    const { return obstacles_; }
    const Obstacle*        findById(int id) const;

private:
    std::vector<Obstacle> obstacles_;
    int next_id_ = 1;
};
