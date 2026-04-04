#pragma once
#include <QDialog>
#include <QTableWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "simulator/sweep_runner.hpp"

class SimulatorNode;

// ---------------------------------------------------------------------------
// ParamConfigDialog
//   Fetches numeric parameters from a live ROS2 node and lets the user mark
//   each one as "Fixed" (constant value every trial) or "Sweep" (start/end/step).
//   Returns a vector<ParamDef> where Fixed params have start == end.
// ---------------------------------------------------------------------------
class ParamConfigDialog : public QDialog {
    Q_OBJECT
public:
    explicit ParamConfigDialog(std::shared_ptr<SimulatorNode> node,
                                const std::vector<ParamDef>& current_params,
                                QWidget* parent = nullptr);

    // Returns the checked params after the dialog is accepted.
    std::vector<ParamDef> getParams() const;

private slots:
    void onFetch();
    void onFilter(const QString& text);

private:
    void populateTable(const std::vector<rclcpp::Parameter>& params);
    void setRowSweepMode(int row, bool sweep);

    std::shared_ptr<SimulatorNode> node_;
    std::vector<ParamDef>          existing_;   // pre-populate checkboxes
    QString                        fetched_node_;

    QLineEdit*    node_edit_;
    QLineEdit*    prefix_edit_;
    QLineEdit*    filter_edit_;
    QTableWidget* table_;
    QLabel*       status_lbl_;
    QPushButton*  ok_btn_;
};
