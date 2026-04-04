#pragma once
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QMouseEvent>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

#include "simulator/map_loader.hpp"
#include "simulator/obstacle_manager.hpp"
#include "simulator/gui/edit_tool.hpp"

class MapView : public QGraphicsView {
    Q_OBJECT
public:
    explicit MapView(QWidget* parent = nullptr);

    void setMap(const MapData& map);
    void updateRobot(double x, double y, double theta, double radius,
                     const std::vector<std::pair<double,double>>& footprint = {});
    void syncObstacles(const std::vector<Obstacle>& obstacles);
    void setTool(EditTool tool);
    void selectObstacle(int id);   // highlight from external source

    // Path drawing
    void clearPath();
    void setPathWaypoints(const std::vector<std::pair<double,double>>& wps);
    std::vector<std::pair<double,double>> getPathWaypoints() const;

    // Coordinate conversions  (requires map to be loaded)
    QPointF worldToScene(double wx, double wy) const;
    void    sceneToWorld(const QPointF& sp, double& wx, double& wy) const;

signals:
    // Emitted whenever path waypoints change (DRAW_PATH tool or clearPath())
    void pathWaypointsChanged(std::vector<std::pair<double,double>> waypoints);

    // Emitted when user completes an add/delete/pose action
    void requestAddStaticCircle(double x, double y);
    void requestAddStaticRect(double x1, double y1, double x2, double y2);
    void requestAddDynamic(double sx, double sy, double ex, double ey);
    void requestDelete(int id);
    void requestSetPose(double x, double y, double theta);
    void obstacleClicked(int id);

protected:
    void mousePressEvent(QMouseEvent* e)   override;
    void mouseMoveEvent(QMouseEvent* e)    override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e)        override;
    void keyPressEvent(QKeyEvent* e)       override;

private:
    void drawObstacle(const Obstacle& obs, bool selected);
    void updateObstacleItem(const Obstacle& obs, bool selected);
    void applyStyle(QAbstractGraphicsShapeItem* item,
                    bool dynamic, bool selected);
    int  idAtScenePos(const QPointF& sp) const;

    QGraphicsScene*      scene_      = nullptr;
    MapData              map_data_;
    EditTool             tool_       = EditTool::SELECT;

    // Robot visualisation — circle (fallback) or polygon footprint
    QGraphicsEllipseItem*  robot_circle_  = nullptr;
    QGraphicsPolygonItem*  robot_polygon_ = nullptr;
    QGraphicsLineItem*     robot_dir_     = nullptr;

    // Obstacle items:  id -> QAbstractGraphicsShapeItem*
    std::unordered_map<int, QAbstractGraphicsShapeItem*> obs_items_;
    // Path lines for dynamic obstacles: id -> QGraphicsLineItem*
    std::unordered_map<int, QGraphicsLineItem*>          dyn_lines_;
    int selected_obs_id_ = -1;

    // Mouse drag state
    bool    pressing_    = false;
    QPointF press_scene_;
    double  press_wx_    = 0.0, press_wy_ = 0.0;

    // Rect-drag preview
    QGraphicsRectItem* drag_preview_ = nullptr;

    // Dynamic obstacle: two-click state
    bool   dyn_waiting_end_ = false;
    double dyn_start_wx_    = 0.0, dyn_start_wy_ = 0.0;
    QGraphicsEllipseItem* dyn_start_marker_ = nullptr;

    // Path drawing (DRAW_PATH tool)
    std::vector<std::pair<double,double>> path_waypoints_;
    QGraphicsPathItem*                    path_line_item_ = nullptr;
    std::vector<QGraphicsEllipseItem*>    path_wp_items_;
    void refreshPathDisplay();
};
