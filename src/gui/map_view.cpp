#include "simulator/gui/map_view.hpp"
#include <QWheelEvent>
#include <QKeyEvent>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <cmath>
#include <unordered_set>

// ---------------------------------------------------------------------------
// Colour palette
static const QColor kColorStaticFill  = QColor(220,  60,  60, 120);
static const QColor kColorStaticLine  = QColor(180,  20,  20);
static const QColor kColorDynFill     = QColor( 60,  60, 220, 120);
static const QColor kColorDynLine     = QColor( 20,  20, 180);
static const QColor kColorPath        = QColor(255, 220,   0);      // yellow
static const QColor kColorPathWp      = QColor(255, 160,   0);      // orange
static const QColor kColorSelected    = QColor(255, 200,   0);
static const QColor kColorRobot       = QColor( 30, 180,  30);
static const QColor kColorRobotDir    = QColor(255, 255,   0);

// ---------------------------------------------------------------------------
MapView::MapView(QWidget* parent)
: QGraphicsView(parent)
{
    scene_ = new QGraphicsScene(this);
    setScene(scene_);
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::NoDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setBackgroundBrush(QBrush(QColor(40, 40, 40)));
    setFocusPolicy(Qt::StrongFocus);
}

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------
QPointF MapView::worldToScene(double wx, double wy) const {
    if (!map_data_.isValid()) return {wx, wy};
    double sx = (wx - map_data_.origin_x) / map_data_.resolution;
    double sy = map_data_.height - 1 - (wy - map_data_.origin_y) / map_data_.resolution;
    return {sx, sy};
}

void MapView::sceneToWorld(const QPointF& sp, double& wx, double& wy) const {
    if (!map_data_.isValid()) { wx = sp.x(); wy = sp.y(); return; }
    wx = sp.x() * map_data_.resolution + map_data_.origin_x;
    wy = (map_data_.height - 1 - sp.y()) * map_data_.resolution + map_data_.origin_y;
}

// ---------------------------------------------------------------------------
void MapView::setMap(const MapData& map) {
    map_data_ = map;
    scene_->clear();
    obs_items_.clear();
    dyn_lines_.clear();
    robot_circle_     = nullptr;
    robot_polygon_    = nullptr;
    robot_dir_        = nullptr;
    dyn_start_marker_ = nullptr;
    drag_preview_     = nullptr;
    path_line_item_   = nullptr;
    path_wp_items_.clear();

    if (!map.isValid()) return;

    QPixmap pm = QPixmap::fromImage(map.image);
    auto* bg = scene_->addPixmap(pm);
    bg->setZValue(-10);
    bg->setData(0, -1);   // id = -1 → not an obstacle

    scene_->setSceneRect(bg->boundingRect());
    fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);

    // Re-draw previously set path in the new coordinate system
    refreshPathDisplay();
}

// ---------------------------------------------------------------------------
// Path drawing helpers
// ---------------------------------------------------------------------------
void MapView::clearPath() {
    path_waypoints_.clear();
    refreshPathDisplay();
    emit pathWaypointsChanged(path_waypoints_);
}

void MapView::setPathWaypoints(const std::vector<std::pair<double,double>>& wps) {
    path_waypoints_ = wps;
    refreshPathDisplay();
}

std::vector<std::pair<double,double>> MapView::getPathWaypoints() const {
    return path_waypoints_;
}

void MapView::refreshPathDisplay() {
    // Remove old path items
    if (path_line_item_) {
        scene_->removeItem(path_line_item_);
        delete path_line_item_;
        path_line_item_ = nullptr;
    }
    for (auto* it : path_wp_items_) { scene_->removeItem(it); delete it; }
    path_wp_items_.clear();

    if (!map_data_.isValid() || path_waypoints_.empty()) return;

    // Polyline between waypoints
    QPainterPath pp;
    QPointF first = worldToScene(path_waypoints_[0].first, path_waypoints_[0].second);
    pp.moveTo(first);
    for (size_t i = 1; i < path_waypoints_.size(); ++i) {
        QPointF p = worldToScene(path_waypoints_[i].first, path_waypoints_[i].second);
        pp.lineTo(p);
    }
    path_line_item_ = scene_->addPath(pp, QPen(kColorPath, 2));
    path_line_item_->setZValue(3);
    path_line_item_->setData(0, -3);

    // Waypoint markers
    constexpr double kWpRadius = 5.0;   // scene pixels
    for (size_t i = 0; i < path_waypoints_.size(); ++i) {
        QPointF sp = worldToScene(path_waypoints_[i].first, path_waypoints_[i].second);
        auto* dot = scene_->addEllipse(
            -kWpRadius, -kWpRadius, 2*kWpRadius, 2*kWpRadius,
            QPen(kColorPath, 1), QBrush(kColorPathWp));
        dot->setPos(sp);
        dot->setZValue(4);
        dot->setData(0, -3);    // not an obstacle
        path_wp_items_.push_back(dot);
    }
}

// ---------------------------------------------------------------------------
void MapView::updateRobot(double x, double y, double theta, double radius,
                           const std::vector<std::pair<double,double>>& footprint)
{
    if (!map_data_.isValid()) return;

    const QPointF sp = worldToScene(x, y);
    const double  rs = radius / map_data_.resolution;

    // ---- Direction arrow (always shown) ----
    if (!robot_dir_) {
        robot_dir_ = scene_->addLine(0, 0, rs, 0, QPen(kColorRobotDir, 2));
        robot_dir_->setZValue(6);
    }
    robot_dir_->setLine(sp.x(), sp.y(),
                        sp.x() + rs * std::cos(theta),
                        sp.y() - rs * std::sin(theta));  // -sin: y-flip

    if (!footprint.empty()) {
        // ---- Polygon footprint ----
        if (robot_circle_) robot_circle_->hide();

        QPolygonF poly;
        for (const auto& [lx, ly] : footprint)
            poly << QPointF(lx / map_data_.resolution,
                           -ly / map_data_.resolution);  // y-flip

        if (!robot_polygon_) {
            robot_polygon_ = scene_->addPolygon(poly);
            robot_polygon_->setPen(QPen(Qt::darkGreen, 2));
            robot_polygon_->setBrush(QBrush(kColorRobot));
            robot_polygon_->setZValue(5);
            robot_polygon_->setData(0, 0);
        } else {
            robot_polygon_->setPolygon(poly);
        }
        robot_polygon_->setPos(sp);
        robot_polygon_->setRotation(-theta * 180.0 / M_PI);  // world CCW → scene CW
        robot_polygon_->show();
    } else {
        // ---- Circle fallback ----
        if (robot_polygon_) robot_polygon_->hide();

        if (!robot_circle_) {
            robot_circle_ = scene_->addEllipse(-rs, -rs, 2*rs, 2*rs);
            robot_circle_->setPen(QPen(Qt::darkGreen, 2));
            robot_circle_->setBrush(QBrush(kColorRobot));
            robot_circle_->setZValue(5);
            robot_circle_->setData(0, 0);
        }
        robot_circle_->setRect(-rs, -rs, 2*rs, 2*rs);
        robot_circle_->setPos(sp);
        robot_circle_->show();
    }
}

// ---------------------------------------------------------------------------
// Obstacle styling
// ---------------------------------------------------------------------------
void MapView::applyStyle(QAbstractGraphicsShapeItem* item,
                          bool dynamic, bool selected) {
    QColor fill = dynamic ? kColorDynFill  : kColorStaticFill;
    QColor line = dynamic ? kColorDynLine  : kColorStaticLine;
    if (selected) line = kColorSelected;
    item->setPen(QPen(line, 2));
    item->setBrush(QBrush(fill));
}

void MapView::drawObstacle(const Obstacle& obs, bool selected) {
    QPointF sp  = worldToScene(obs.x, obs.y);
    QAbstractGraphicsShapeItem* item = nullptr;

    if (obs.shape == ObstacleShape::CIRCLE) {
        double rs = obs.radius / map_data_.resolution;
        auto*  e  = new QGraphicsEllipseItem(-rs, -rs, 2*rs, 2*rs);
        e->setPos(sp);
        scene_->addItem(e);
        item = e;
    } else {
        double ws = obs.width  / map_data_.resolution;
        double hs = obs.height / map_data_.resolution;
        auto*  r  = new QGraphicsRectItem(-ws/2, -hs/2, ws, hs);
        r->setPos(sp);
        r->setRotation(-obs.angle * 180.0 / M_PI);
        scene_->addItem(r);
        item = r;
    }

    applyStyle(item, obs.dynamic, selected);
    item->setZValue(2);
    item->setData(0, obs.id);
    obs_items_[obs.id] = item;

    // Draw path line for dynamic obstacles
    if (obs.dynamic) {
        QPointF s_start = worldToScene(obs.start_x, obs.start_y);
        QPointF s_end   = worldToScene(obs.end_x,   obs.end_y);
        QPen dash(kColorDynLine, 1, Qt::DashLine);
        auto* line = scene_->addLine(s_start.x(), s_start.y(),
                                     s_end.x(),   s_end.y(), dash);
        line->setZValue(1);
        line->setData(0, -2);  // not an obstacle
        dyn_lines_[obs.id] = line;
    }
}

void MapView::updateObstacleItem(const Obstacle& obs, bool selected) {
    auto it = obs_items_.find(obs.id);
    if (it == obs_items_.end()) { drawObstacle(obs, selected); return; }

    QPointF sp = worldToScene(obs.x, obs.y);
    auto* item = it->second;

    if (obs.shape == ObstacleShape::CIRCLE) {
        auto* e = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
        if (e) {
            double rs = obs.radius / map_data_.resolution;
            e->setRect(-rs, -rs, 2*rs, 2*rs);
            e->setPos(sp);
        }
    } else {
        auto* r = qgraphicsitem_cast<QGraphicsRectItem*>(item);
        if (r) {
            double ws = obs.width  / map_data_.resolution;
            double hs = obs.height / map_data_.resolution;
            r->setRect(-ws/2, -hs/2, ws, hs);
            r->setPos(sp);
            r->setRotation(-obs.angle * 180.0 / M_PI);
        }
    }
    applyStyle(item, obs.dynamic, selected);

    // Update dynamic path line
    if (obs.dynamic) {
        auto lit = dyn_lines_.find(obs.id);
        if (lit != dyn_lines_.end()) {
            QPointF s_start = worldToScene(obs.start_x, obs.start_y);
            QPointF s_end   = worldToScene(obs.end_x,   obs.end_y);
            lit->second->setLine(s_start.x(), s_start.y(),
                                 s_end.x(),   s_end.y());
        }
    }
}

// ---------------------------------------------------------------------------
void MapView::syncObstacles(const std::vector<Obstacle>& obstacles) {
    std::unordered_set<int> active;
    for (const auto& o : obstacles) active.insert(o.id);

    // Remove stale items
    for (auto it = obs_items_.begin(); it != obs_items_.end(); ) {
        if (!active.count(it->first)) {
            scene_->removeItem(it->second);
            delete it->second;
            it = obs_items_.erase(it);
            // Also remove path line
            auto lit = dyn_lines_.find(it == obs_items_.end() ? -1 : it->first);
            if (lit != dyn_lines_.end()) {
                scene_->removeItem(lit->second);
                delete lit->second;
                dyn_lines_.erase(lit);
            }
        } else {
            ++it;
        }
    }
    // Also clean up dyn_lines_ for removed obstacles
    for (auto it = dyn_lines_.begin(); it != dyn_lines_.end(); ) {
        if (!active.count(it->first)) {
            scene_->removeItem(it->second);
            delete it->second;
            it = dyn_lines_.erase(it);
        } else {
            ++it;
        }
    }

    // Add / update
    for (const auto& obs : obstacles) {
        bool sel = (obs.id == selected_obs_id_);
        if (obs_items_.count(obs.id) == 0)
            drawObstacle(obs, sel);
        else
            updateObstacleItem(obs, sel);
    }
}

void MapView::selectObstacle(int id) {
    // Deselect previous
    auto prev = obs_items_.find(selected_obs_id_);
    if (prev != obs_items_.end()) {
        // Find the obstacle to know if dynamic
        applyStyle(prev->second, false, false);  // simplified: reset without dynamic info
    }
    selected_obs_id_ = id;
    auto cur = obs_items_.find(id);
    if (cur != obs_items_.end()) {
        applyStyle(cur->second, false, true);
    }
}

void MapView::setTool(EditTool tool) {
    tool_ = tool;
    // Cancel any in-progress dynamic placement
    if (tool != EditTool::ADD_DYNAMIC && dyn_waiting_end_) {
        dyn_waiting_end_ = false;
        if (dyn_start_marker_) {
            scene_->removeItem(dyn_start_marker_);
            delete dyn_start_marker_;
            dyn_start_marker_ = nullptr;
        }
    }
    if (drag_preview_) {
        scene_->removeItem(drag_preview_);
        delete drag_preview_;
        drag_preview_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Helper: find obstacle id at scene pos (returns -1 if none)
// ---------------------------------------------------------------------------
int MapView::idAtScenePos(const QPointF& sp) const {
    const auto items = scene_->items(sp);
    for (auto* item : items) {
        int id = item->data(0).toInt();
        if (id > 0) return id;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Mouse events
// ---------------------------------------------------------------------------
void MapView::mousePressEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) {
        QGraphicsView::mousePressEvent(e);
        return;
    }
    QPointF sp = mapToScene(e->pos());
    double wx, wy;
    sceneToWorld(sp, wx, wy);
    pressing_   = true;
    press_scene_ = sp;
    press_wx_ = wx; press_wy_ = wy;

    switch (tool_) {
        case EditTool::SELECT: {
            int id = idAtScenePos(sp);
            if (id > 0) emit obstacleClicked(id);
            break;
        }
        case EditTool::DELETE_OBSTACLE: {
            int id = idAtScenePos(sp);
            if (id > 0) emit requestDelete(id);
            break;
        }
        case EditTool::ADD_STATIC_CIRCLE:
            // Will emit on release
            break;
        case EditTool::ADD_STATIC_RECT: {
            // Start drag preview
            drag_preview_ = scene_->addRect(sp.x(), sp.y(), 0, 0,
                                            QPen(Qt::white, 1, Qt::DashLine));
            drag_preview_->setZValue(10);
            break;
        }
        case EditTool::ADD_DYNAMIC: {
            if (!dyn_waiting_end_) {
                // First click → place start marker
                dyn_start_wx_ = wx; dyn_start_wy_ = wy;
                dyn_waiting_end_ = true;
                dyn_start_marker_ = scene_->addEllipse(sp.x()-5, sp.y()-5, 10, 10,
                                                        QPen(Qt::blue), QBrush(Qt::blue));
                dyn_start_marker_->setZValue(10);
            } else {
                // Second click → emit with both endpoints
                if (dyn_start_marker_) {
                    scene_->removeItem(dyn_start_marker_);
                    delete dyn_start_marker_;
                    dyn_start_marker_ = nullptr;
                }
                dyn_waiting_end_ = false;
                emit requestAddDynamic(dyn_start_wx_, dyn_start_wy_, wx, wy);
            }
            break;
        }
        case EditTool::SET_ROBOT_POSE:
            // Position set on press; direction set on release (after drag)
            break;

        case EditTool::DRAW_PATH:
            if (e->button() == Qt::LeftButton) {
                path_waypoints_.push_back({wx, wy});
                refreshPathDisplay();
                emit pathWaypointsChanged(path_waypoints_);
            } else if (e->button() == Qt::RightButton) {
                // Undo last waypoint
                if (!path_waypoints_.empty()) {
                    path_waypoints_.pop_back();
                    refreshPathDisplay();
                    emit pathWaypointsChanged(path_waypoints_);
                }
            }
            break;
    }
}

void MapView::mouseMoveEvent(QMouseEvent* e) {
    QPointF sp = mapToScene(e->pos());

    if (pressing_ && tool_ == EditTool::ADD_STATIC_RECT && drag_preview_) {
        double x1 = std::min(press_scene_.x(), sp.x());
        double y1 = std::min(press_scene_.y(), sp.y());
        double x2 = std::max(press_scene_.x(), sp.x());
        double y2 = std::max(press_scene_.y(), sp.y());
        drag_preview_->setRect(x1, y1, x2 - x1, y2 - y1);
    }
    QGraphicsView::mouseMoveEvent(e);
}

void MapView::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) {
        QGraphicsView::mouseReleaseEvent(e);
        return;
    }
    QPointF sp = mapToScene(e->pos());
    double wx, wy;
    sceneToWorld(sp, wx, wy);

    if (!pressing_) return;
    pressing_ = false;

    switch (tool_) {
        case EditTool::ADD_STATIC_CIRCLE:
            emit requestAddStaticCircle(press_wx_, press_wy_);
            break;

        case EditTool::ADD_STATIC_RECT: {
            if (drag_preview_) {
                scene_->removeItem(drag_preview_);
                delete drag_preview_;
                drag_preview_ = nullptr;
            }
            double dx = wx - press_wx_, dy = wy - press_wy_;
            if (std::hypot(dx, dy) > 0.02) {
                emit requestAddStaticRect(press_wx_, press_wy_, wx, wy);
            }
            break;
        }
        case EditTool::SET_ROBOT_POSE: {
            double dx = sp.x() - press_scene_.x();
            double dy = sp.y() - press_scene_.y();
            double theta = 0.0;
            if (std::hypot(dx, dy) > 3.0) {
                // Direction from drag (note: scene y is down → negate dy)
                theta = std::atan2(-dy, dx);
            }
            emit requestSetPose(press_wx_, press_wy_, theta);
            break;
        }
        default:
            break;
    }
    QGraphicsView::mouseReleaseEvent(e);
}

void MapView::wheelEvent(QWheelEvent* e) {
    double factor = (e->angleDelta().y() > 0) ? 1.15 : (1.0 / 1.15);
    scale(factor, factor);
}

void MapView::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Escape) {
        if (dyn_waiting_end_) {
            dyn_waiting_end_ = false;
            if (dyn_start_marker_) {
                scene_->removeItem(dyn_start_marker_);
                delete dyn_start_marker_;
                dyn_start_marker_ = nullptr;
            }
        }
    } else if (e->key() == Qt::Key_Backspace || e->key() == Qt::Key_Delete) {
        // Undo last path waypoint when in DRAW_PATH mode
        if (tool_ == EditTool::DRAW_PATH && !path_waypoints_.empty()) {
            path_waypoints_.pop_back();
            refreshPathDisplay();
            emit pathWaypointsChanged(path_waypoints_);
        }
    }
    QGraphicsView::keyPressEvent(e);
}
