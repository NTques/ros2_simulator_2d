#include "simulator/obstacle_manager.hpp"
#include <algorithm>
#include <cmath>

int ObstacleManager::addObstacle(Obstacle obs) {
    obs.id = next_id_++;
    // Initialise dynamic start position to current position
    if (obs.dynamic) {
        obs.start_x = obs.x;
        obs.start_y = obs.y;
    }
    obstacles_.push_back(obs);
    return obs.id;
}

void ObstacleManager::removeObstacle(int id) {
    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
                       [id](const Obstacle& o) { return o.id == id; }),
        obstacles_.end());
}

void ObstacleManager::updateObstacle(const Obstacle& obs) {
    for (auto& o : obstacles_) {
        if (o.id == obs.id) { o = obs; return; }
    }
}

const Obstacle* ObstacleManager::findById(int id) const {
    for (const auto& o : obstacles_) {
        if (o.id == id) return &o;
    }
    return nullptr;
}

void ObstacleManager::clear() {
    obstacles_.clear();
}

void ObstacleManager::updateDynamic(double dt) {
    for (auto& obs : obstacles_) {
        if (!obs.dynamic) continue;

        double tx = (obs.dir > 0) ? obs.end_x : obs.start_x;
        double ty = (obs.dir > 0) ? obs.end_y : obs.start_y;

        double dx   = tx - obs.x;
        double dy   = ty - obs.y;
        double dist = std::hypot(dx, dy);

        if (dist < 1e-6) {
            obs.dir = -obs.dir;
            continue;
        }

        double step = obs.speed * dt;
        if (step >= dist) {
            obs.x   = tx;
            obs.y   = ty;
            obs.dir = -obs.dir;
        } else {
            obs.x += (dx / dist) * step;
            obs.y += (dy / dist) * step;
        }
    }
}
