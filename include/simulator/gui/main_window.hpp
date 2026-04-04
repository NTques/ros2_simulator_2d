#pragma once
#include <QMainWindow>
#include <QTimer>
#include <QListWidget>
#include <QLabel>
#include <QActionGroup>
#include <QSlider>
#include <QDockWidget>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <memory>

#include "simulator/simulator_node.hpp"
#include "simulator/gui/edit_tool.hpp"
#include "simulator/gui/sweep_panel.hpp"

class MapView;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(std::shared_ptr<SimulatorNode> node,
                        QWidget* parent = nullptr);

private slots:
    void onOpenMap();
    void onLoadURDF();
    void onPhysicsUpdate();
    void onSpeedChanged(int value);       // slider
    void onObstacleListClicked(QListWidgetItem* item);
    void onRemoveObstacle();
    void onApplyProperties();

    // Signals from MapView
    void onAddStaticCircle(double x, double y);
    void onAddStaticRect(double x1, double y1, double x2, double y2);
    void onAddDynamic(double sx, double sy, double ex, double ey);
    void onDeleteObstacle(int id);
    void onSetPose(double x, double y, double theta);
    void onObstacleClicked(int id);

private:
    void setupToolbar();
    void setupDockPanel();
    void refreshObstacleList();
    void populateProperties(int id);
    void clearProperties();

    std::shared_ptr<SimulatorNode> node_;

    MapView*       map_view_    = nullptr;
    SweepPanel*    sweep_panel_ = nullptr;
    QTimer*        phys_timer_  = nullptr;

    // Dock panel widgets
    QListWidget*    obs_list_       = nullptr;
    QLabel*         props_title_    = nullptr;
    QDoubleSpinBox* prop_x_         = nullptr;
    QDoubleSpinBox* prop_y_         = nullptr;
    QDoubleSpinBox* prop_r_         = nullptr;
    QDoubleSpinBox* prop_w_         = nullptr;
    QDoubleSpinBox* prop_h_         = nullptr;
    QDoubleSpinBox* prop_angle_     = nullptr;
    QCheckBox*      prop_dynamic_   = nullptr;
    QDoubleSpinBox* prop_ex_        = nullptr;
    QDoubleSpinBox* prop_ey_        = nullptr;
    QDoubleSpinBox* prop_speed_     = nullptr;
    QPushButton*    btn_apply_      = nullptr;
    QWidget*        rect_params_    = nullptr;
    QWidget*        dyn_params_     = nullptr;

    QLabel*         status_label_   = nullptr;
    QLabel*         speed_label_    = nullptr;
    QActionGroup*   tool_group_     = nullptr;

    int selected_obs_id_ = -1;
};
