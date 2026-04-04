#include "simulator/gui/sweep_panel.hpp"
#include "simulator/gui/param_config_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QBrush>
#include <QColor>
#include <algorithm>
#include <limits>
#include <cmath>

// ---------------------------------------------------------------------------
SweepPanel::SweepPanel(std::shared_ptr<SimulatorNode> node, QWidget* parent)
: QWidget(parent), node_(node)
{
    runner_ = std::make_unique<SweepRunner>(node_);

    connect(runner_.get(), &SweepRunner::trialFinished,
            this, &SweepPanel::onTrialFinished);
    connect(runner_.get(), &SweepRunner::sweepFinished,
            this, &SweepPanel::onSweepFinished);
    connect(runner_.get(), &SweepRunner::progressChanged,
            this, [this](int cur, int total) {
                progress_->setMaximum(total);
                progress_->setValue(cur);
            });
    connect(runner_.get(), &SweepRunner::statusMessage,
            this, [this](const QString& msg) { status_lbl_->setText(msg); });

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(6, 6, 6, 6);
    main_layout->setSpacing(6);

    // ---- Control row: Start/Stop + timing settings ----
    auto* ctrl_row = new QHBoxLayout;
    ctrl_row->setSpacing(6);

    btn_run_  = new QPushButton("▶  Start Sweep");
    btn_stop_ = new QPushButton("■  Stop");
    btn_stop_->setEnabled(false);
    btn_run_->setMinimumWidth(130);

    sc_timeout_ = new QDoubleSpinBox;
    sc_timeout_->setRange(1.0, 600.0);
    sc_timeout_->setValue(30.0);
    sc_timeout_->setSuffix(" s");
    sc_timeout_->setToolTip("Duration of each trial");

    sc_settle_ = new QDoubleSpinBox;
    sc_settle_->setRange(0.1, 30.0);
    sc_settle_->setValue(0.5);
    sc_settle_->setSuffix(" s");
    sc_settle_->setToolTip("Wait time after applying parameters before trial begins");

    sc_ctrl_id_ = new QLineEdit("FollowPath");
    sc_ctrl_id_->setMaximumWidth(140);
    sc_ctrl_id_->setToolTip("Controller plugin name passed to the follow_path action server");

    ctrl_row->addWidget(btn_run_);
    ctrl_row->addWidget(btn_stop_);
    ctrl_row->addSpacing(12);
    ctrl_row->addWidget(new QLabel("Timeout:"));
    ctrl_row->addWidget(sc_timeout_);
    ctrl_row->addWidget(new QLabel("Settle:"));
    ctrl_row->addWidget(sc_settle_);
    ctrl_row->addWidget(new QLabel("Controller:"));
    ctrl_row->addWidget(sc_ctrl_id_);
    ctrl_row->addStretch();
    main_layout->addLayout(ctrl_row);

    // ---- Progress + status ----
    progress_ = new QProgressBar;
    progress_->setTextVisible(true);
    progress_->setFormat("%v / %m");
    main_layout->addWidget(progress_);

    status_lbl_ = new QLabel("Click \"Start Sweep\" to configure parameters and begin.");
    status_lbl_->setStyleSheet("color: #aaaaaa; font-style: italic;");
    main_layout->addWidget(status_lbl_);

    // ---- Results header row ----
    auto* res_hdr = new QHBoxLayout;
    res_hdr->addWidget(new QLabel("Sort by:"));
    sort_metric_ = new QComboBox;
    sort_metric_->addItems({"elapsed_time(s)", "path_length(m)",
                             "smoothness", "tracking_err_rms(m)", "tracking_err_max(m)"});
    connect(sort_metric_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SweepPanel::onSortMetricChanged);
    res_hdr->addWidget(sort_metric_);
    res_hdr->addStretch();
    auto* btn_export = new QPushButton("Export CSV");
    connect(btn_export, &QPushButton::clicked, this, &SweepPanel::onExportCSV);
    res_hdr->addWidget(btn_export);
    main_layout->addLayout(res_hdr);

    // ---- Results table ----
    results_table_ = new QTableWidget(0, 0);
    results_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    results_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    results_table_->setAlternatingRowColors(true);
    results_table_->horizontalHeader()->setStretchLastSection(true);
    main_layout->addWidget(results_table_, 1);

    connect(btn_run_,  &QPushButton::clicked, this, &SweepPanel::onRun);
    connect(btn_stop_, &QPushButton::clicked, this, &SweepPanel::onStop);
}

// ---------------------------------------------------------------------------
void SweepPanel::tick(double real_dt, double sim_dt, const RobotState& robot) {
    runner_->tick(real_dt, sim_dt, robot);
}

void SweepPanel::setPathWaypoints(const std::vector<std::pair<double,double>>& wps) {
    path_waypoints_ = wps;
}

// ---------------------------------------------------------------------------
void SweepPanel::onRun() {
    // Require a path to be drawn before opening the dialog
    if (path_waypoints_.empty()) {
        status_lbl_->setText("Draw a path on the map first (Draw Path tool), then start the sweep.");
        return;
    }

    // Open parameter configuration dialog
    ParamConfigDialog dlg(node_, current_params_, this);
    if (dlg.exec() != QDialog::Accepted) return;

    current_params_ = dlg.getParams();
    if (current_params_.empty()) {
        status_lbl_->setText("No parameters selected. Check at least one parameter in the dialog.");
        return;
    }

    // Build scenario from UI settings
    ScenarioConfig sc;
    sc.timeout_sec   = sc_timeout_->value();
    sc.settle_sec    = sc_settle_->value();
    sc.controller_id = sc_ctrl_id_->text().trimmed().toStdString();
    if (sc.controller_id.empty()) sc.controller_id = "FollowPath";
    for (const auto& [wx, wy] : path_waypoints_)
        sc.reference_path.push_back({wx, wy});

    runner_->setConfig(current_params_, sc);
    runner_->start();

    // Build results table columns: one per param + metric columns
    QStringList headers;
    for (const auto& p : current_params_)
        headers << QString::fromStdString(p.param_name);
    headers << "elapsed(s)" << "path_len(m)" << "smoothness"
            << "track_rms(m)" << "track_max(m)";
    results_table_->setColumnCount(headers.size());
    results_table_->setHorizontalHeaderLabels(headers);
    results_table_->setRowCount(0);
    results_table_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);

    btn_run_->setEnabled(false);
    btn_stop_->setEnabled(true);
}

void SweepPanel::onStop() {
    runner_->stop();
    btn_run_->setEnabled(true);
    btn_stop_->setEnabled(false);
}

// ---------------------------------------------------------------------------
void SweepPanel::onTrialFinished(TrialResult res) {
    const int row = results_table_->rowCount();
    results_table_->insertRow(row);
    int col = 0;

    auto cell = [&](const QString& txt) {
        auto* item = new QTableWidgetItem(txt);
        item->setTextAlignment(Qt::AlignCenter);
        results_table_->setItem(row, col++, item);
    };

    for (double v : res.param_values)
        cell(QString::number(v, 'g', 5));

    cell(QString::number(res.elapsed_sec,  'f', 2));
    cell(QString::number(res.path_length,  'f', 2));
    cell(QString::number(res.smoothness,   'f', 4));
    cell(res.tracking_err_rms >= 0 ? QString::number(res.tracking_err_rms, 'f', 3) : "N/A");
    cell(res.tracking_err_max >= 0 ? QString::number(res.tracking_err_max, 'f', 3) : "N/A");

    results_table_->scrollToBottom();
}

void SweepPanel::onSweepFinished() {
    btn_run_->setEnabled(true);
    btn_stop_->setEnabled(false);
    rebuildResultsTable();
}

// ---------------------------------------------------------------------------
void SweepPanel::rebuildResultsTable() {
    const auto& raw = runner_->results();
    if (raw.empty()) return;

    const int metric_idx = sort_metric_->currentIndex();

    auto getMetric = [&](const TrialResult& r) -> double {
        switch (metric_idx) {
            case 0: return r.elapsed_sec;
            case 1: return r.path_length;
            case 2: return r.smoothness;
            case 3: return r.tracking_err_rms >= 0
                           ? r.tracking_err_rms
                           : std::numeric_limits<double>::infinity();
            case 4: return r.tracking_err_max >= 0
                           ? r.tracking_err_max
                           : std::numeric_limits<double>::infinity();
            default: return r.elapsed_sec;
        }
    };

    auto sorted = raw;
    std::sort(sorted.begin(), sorted.end(),
              [&](const TrialResult& a, const TrialResult& b) {
                  return getMetric(a) < getMetric(b);
              });

    results_table_->clearContents();
    results_table_->setRowCount(0);

    for (int ri = 0; ri < static_cast<int>(sorted.size()); ++ri) {
        const auto& res = sorted[ri];
        const int row = results_table_->rowCount();
        results_table_->insertRow(row);
        int col = 0;

        const bool is_best = (ri == 0);
        const QColor bg = is_best ? QColor(0, 100, 0) : QColor();

        auto cell = [&](const QString& txt) {
            auto* item = new QTableWidgetItem(txt);
            item->setTextAlignment(Qt::AlignCenter);
            if (bg.isValid()) {
                item->setBackground(QBrush(bg));
                item->setForeground(QBrush(Qt::white));
            }
            results_table_->setItem(row, col++, item);
        };

        for (double v : res.param_values)
            cell(QString::number(v, 'g', 5));

        cell(QString::number(res.elapsed_sec,  'f', 2));
        cell(QString::number(res.path_length,  'f', 2));
        cell(QString::number(res.smoothness,   'f', 4));
        cell(res.tracking_err_rms >= 0 ? QString::number(res.tracking_err_rms, 'f', 3) : "N/A");
        cell(res.tracking_err_max >= 0 ? QString::number(res.tracking_err_max, 'f', 3) : "N/A");
    }
}

void SweepPanel::onSortMetricChanged() {
    if (!runner_->isRunning()) rebuildResultsTable();
}

// ---------------------------------------------------------------------------
void SweepPanel::onExportCSV() {
    QString path = QFileDialog::getSaveFileName(
        this, "Export Results CSV", "sweep_results.csv", "CSV (*.csv)");
    if (path.isEmpty()) return;

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&f);

    QStringList hdr;
    for (int c = 0; c < results_table_->columnCount(); ++c) {
        auto* h = results_table_->horizontalHeaderItem(c);
        hdr << (h ? h->text() : "");
    }
    out << hdr.join(",") << "\n";

    for (int r = 0; r < results_table_->rowCount(); ++r) {
        QStringList row;
        for (int c = 0; c < results_table_->columnCount(); ++c) {
            auto* item = results_table_->item(r, c);
            row << (item ? item->text() : "");
        }
        out << row.join(",") << "\n";
    }
}
