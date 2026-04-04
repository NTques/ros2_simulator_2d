#pragma once
#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <memory>
#include <vector>
#include <utility>
#include "simulator/sweep_runner.hpp"
#include "simulator/simulator_node.hpp"

class SweepPanel : public QWidget {
    Q_OBJECT
public:
    explicit SweepPanel(std::shared_ptr<SimulatorNode> node,
                        QWidget* parent = nullptr);

    // Called from MainWindow::onPhysicsUpdate() at ~50 Hz
    void tick(double real_dt, double sim_dt, const RobotState& robot);

    // Called by MainWindow when the drawn map path changes (used as reference path)
    void setPathWaypoints(const std::vector<std::pair<double,double>>& wps);

private slots:
    void onRun();
    void onStop();
    void onTrialFinished(TrialResult result);
    void onSweepFinished();
    void onExportCSV();
    void onSortMetricChanged();

private:
    void rebuildResultsTable();

    std::shared_ptr<SimulatorNode> node_;
    std::unique_ptr<SweepRunner>   runner_;

    // Parameters from the last accepted ParamConfigDialog
    std::vector<ParamDef> current_params_;

    // Map path waypoints used as reference path for cross-track error
    std::vector<std::pair<double,double>> path_waypoints_;

    // Settings
    QDoubleSpinBox* sc_timeout_ = nullptr;
    QDoubleSpinBox* sc_settle_  = nullptr;
    QLineEdit*      sc_ctrl_id_ = nullptr;

    // Controls
    QPushButton*  btn_run_  = nullptr;
    QPushButton*  btn_stop_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QLabel*       status_lbl_ = nullptr;

    // Results
    QTableWidget* results_table_ = nullptr;
    QComboBox*    sort_metric_   = nullptr;
};
