#include "simulator/gui/param_config_dialog.hpp"
#include "simulator/simulator_node.hpp"
#include <rclcpp/parameter_client.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QApplication>
#include <cmath>

// ---------------------------------------------------------------------------
// Column indices
// ---------------------------------------------------------------------------
static constexpr int COL_INCLUDE = 0;
static constexpr int COL_NAME    = 1;
static constexpr int COL_CURRENT = 2;
static constexpr int COL_MODE    = 3;
static constexpr int COL_START   = 4;
static constexpr int COL_END     = 5;
static constexpr int COL_STEP    = 6;
static constexpr int COL_COUNT   = 7;

// ---------------------------------------------------------------------------
ParamConfigDialog::ParamConfigDialog(std::shared_ptr<SimulatorNode> node,
                                      const std::vector<ParamDef>& current_params,
                                      QWidget* parent)
: QDialog(parent), node_(node), existing_(current_params)
{
    setWindowTitle("Parameter Configuration");
    setMinimumSize(880, 580);
    resize(1000, 650);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(6);

    // ---- Connection group ----
    auto* conn_grp = new QGroupBox("ROS2 Node Connection");
    auto* conn_lay = new QHBoxLayout(conn_grp);
    conn_lay->setSpacing(8);

    conn_lay->addWidget(new QLabel("Node:"));
    node_edit_ = new QLineEdit("controller_server");
    node_edit_->setMinimumWidth(180);
    node_edit_->setToolTip("ROS2 node name to fetch parameters from");
    conn_lay->addWidget(node_edit_);

    conn_lay->addSpacing(12);
    conn_lay->addWidget(new QLabel("Prefix filter:"));
    prefix_edit_ = new QLineEdit("FollowPath");
    prefix_edit_->setMinimumWidth(130);
    prefix_edit_->setToolTip(
        "Only fetch parameters whose name starts with this prefix.\n"
        "Leave empty to fetch all numeric parameters from the node.");
    conn_lay->addWidget(prefix_edit_);

    auto* fetch_btn = new QPushButton("  Fetch Parameters  ");
    fetch_btn->setDefault(true);
    conn_lay->addWidget(fetch_btn);
    conn_lay->addStretch();

    status_lbl_ = new QLabel("Enter node name and press Fetch.");
    status_lbl_->setStyleSheet("color: #aaaaaa; font-style: italic;");
    conn_lay->addWidget(status_lbl_);

    main_layout->addWidget(conn_grp);

    // ---- Filter row ----
    auto* filter_row = new QHBoxLayout;
    filter_row->addWidget(new QLabel("Search:"));
    filter_edit_ = new QLineEdit;
    filter_edit_->setPlaceholderText("Filter parameter names...");
    filter_edit_->setClearButtonEnabled(true);
    filter_row->addWidget(filter_edit_);
    main_layout->addLayout(filter_row);

    // ---- Table ----
    table_ = new QTableWidget(0, COL_COUNT);
    table_->setHorizontalHeaderLabels(
        {"✓", "Parameter", "Current", "Mode", "Value / Start", "End", "Step"});
    auto* hdr = table_->horizontalHeader();
    hdr->setSectionResizeMode(COL_INCLUDE, QHeaderView::ResizeToContents);
    hdr->setSectionResizeMode(COL_NAME,    QHeaderView::Stretch);
    hdr->setSectionResizeMode(COL_CURRENT, QHeaderView::ResizeToContents);
    hdr->setSectionResizeMode(COL_MODE,    QHeaderView::ResizeToContents);
    hdr->setSectionResizeMode(COL_START,   QHeaderView::ResizeToContents);
    hdr->setSectionResizeMode(COL_END,     QHeaderView::ResizeToContents);
    hdr->setSectionResizeMode(COL_STEP,    QHeaderView::ResizeToContents);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setAlternatingRowColors(true);
    main_layout->addWidget(table_, 1);

    // ---- Dialog buttons ----
    auto* btn_box = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    ok_btn_ = btn_box->button(QDialogButtonBox::Ok);
    ok_btn_->setEnabled(false);
    ok_btn_->setText("OK  (Apply to sweep)");
    connect(btn_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btn_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    main_layout->addWidget(btn_box);

    // ---- Signal connections ----
    connect(fetch_btn,   &QPushButton::clicked,       this, &ParamConfigDialog::onFetch);
    connect(filter_edit_, &QLineEdit::textChanged,    this, &ParamConfigDialog::onFilter);
    // Allow pressing Enter in node/prefix fields to trigger fetch
    connect(node_edit_,   &QLineEdit::returnPressed,  this, &ParamConfigDialog::onFetch);
    connect(prefix_edit_, &QLineEdit::returnPressed,  this, &ParamConfigDialog::onFetch);
}

// ---------------------------------------------------------------------------
void ParamConfigDialog::onFetch()
{
    const QString node_name = node_edit_->text().trimmed();
    if (node_name.isEmpty()) {
        status_lbl_->setText("Enter a node name first.");
        return;
    }

    status_lbl_->setText("Connecting...");
    QApplication::processEvents();

    // Use a dedicated lightweight node so that spin_until_future_complete
    // does NOT try to spin the simulator node (which is already in ros_thread_).
    auto client_node = rclcpp::Node::make_shared(
        "__param_fetch_client__",
        rclcpp::NodeOptions()
            .start_parameter_services(false)
            .start_parameter_event_publisher(false));
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        client_node, node_name.toStdString());

    if (!client->wait_for_service(std::chrono::milliseconds(1500))) {
        status_lbl_->setText(
            QString("Service unavailable — is /%1 running?").arg(node_name));
        return;
    }

    // Build prefix list
    std::vector<std::string> prefixes;
    const QString prefix = prefix_edit_->text().trimmed();
    if (!prefix.isEmpty()) prefixes.push_back(prefix.toStdString());

    status_lbl_->setText("Listing parameters...");
    QApplication::processEvents();

    const auto list_res = client->list_parameters(prefixes, /*depth=*/10);
    if (list_res.names.empty()) {
        status_lbl_->setText("No parameters found — try a different prefix or leave it empty.");
        return;
    }

    status_lbl_->setText(
        QString("Fetching %1 parameter values...").arg(list_res.names.size()));
    QApplication::processEvents();

    const auto params = client->get_parameters(list_res.names);

    fetched_node_ = node_name;
    populateTable(params);

    status_lbl_->setText(
        QString("Fetched %1 numeric parameter(s) from  %2")
        .arg(table_->rowCount()).arg(node_name));
    ok_btn_->setEnabled(true);
}

// ---------------------------------------------------------------------------
void ParamConfigDialog::populateTable(const std::vector<rclcpp::Parameter>& params)
{
    table_->setRowCount(0);

    for (const auto& p : params) {
        const auto type = p.get_type();
        if (type != rclcpp::ParameterType::PARAMETER_DOUBLE &&
            type != rclcpp::ParameterType::PARAMETER_INTEGER) continue;

        const double cur_val = (type == rclcpp::ParameterType::PARAMETER_DOUBLE)
            ? p.as_double() : static_cast<double>(p.as_int());
        const double default_step =
            (type == rclcpp::ParameterType::PARAMETER_INTEGER) ? 1.0 : 0.1;

        const int row = table_->rowCount();
        table_->insertRow(row);

        // --- Col 0: Include checkbox ---
        auto* inc = new QTableWidgetItem;
        inc->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        inc->setCheckState(Qt::Unchecked);

        // --- Col 1: Name (read-only) ---
        auto* name_item = new QTableWidgetItem(QString::fromStdString(p.get_name()));
        name_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

        // --- Col 2: Current value (read-only) ---
        const QString cur_str = (type == rclcpp::ParameterType::PARAMETER_INTEGER)
            ? QString::number(p.as_int())
            : QString::number(cur_val, 'g', 6);
        auto* cur_item = new QTableWidgetItem(cur_str);
        cur_item->setFlags(Qt::ItemIsEnabled);
        cur_item->setForeground(QColor("#888888"));

        table_->setItem(row, COL_INCLUDE, inc);
        table_->setItem(row, COL_NAME,    name_item);
        table_->setItem(row, COL_CURRENT, cur_item);

        // --- Col 3: Mode combo ---
        auto* mode_cb = new QComboBox;
        mode_cb->addItems({"Fixed", "Sweep"});
        mode_cb->setToolTip("Fixed: same value every trial\nSweep: varied from Start to End");

        // --- Spinboxes ---
        auto makeSpin = [](double lo, double hi, double val, double step, int dec) {
            auto* sb = new QDoubleSpinBox;
            sb->setRange(lo, hi);
            sb->setDecimals(dec);
            sb->setSingleStep(step);
            sb->setValue(val);
            return sb;
        };

        auto* sb_start = makeSpin(-1e6, 1e6, cur_val, default_step, 6);
        auto* sb_end   = makeSpin(-1e6, 1e6, cur_val, default_step, 6);
        auto* sb_step  = makeSpin(1e-9, 1e6, default_step, 0.01, 6);
        sb_end->setEnabled(false);
        sb_step->setEnabled(false);

        // --- Pre-populate from existing params ---
        const std::string pname = p.get_name();
        for (const auto& ex : existing_) {
            if (ex.param_name != pname) continue;
            inc->setCheckState(Qt::Checked);
            sb_start->setValue(ex.start);
            const bool is_sweep = std::abs(ex.end - ex.start) > 1e-12;
            if (is_sweep) {
                mode_cb->setCurrentIndex(1);
                sb_end->setValue(ex.end);
                sb_step->setValue(ex.step);
                sb_end->setEnabled(true);
                sb_step->setEnabled(true);
            }
            break;
        }

        // Connect mode change → enable/disable End & Step
        connect(mode_cb, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this, row](int idx) { setRowSweepMode(row, idx == 1); });

        table_->setCellWidget(row, COL_MODE,  mode_cb);
        table_->setCellWidget(row, COL_START, sb_start);
        table_->setCellWidget(row, COL_END,   sb_end);
        table_->setCellWidget(row, COL_STEP,  sb_step);
    }
}

// ---------------------------------------------------------------------------
void ParamConfigDialog::setRowSweepMode(int row, bool sweep)
{
    auto* sb_start = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_START));
    auto* sb_end   = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_END));
    auto* sb_step  = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_STEP));
    if (!sb_end || !sb_step) return;

    sb_end->setEnabled(sweep);
    sb_step->setEnabled(sweep);
    // In Fixed mode, mirror Start → End so the value is obvious
    if (!sweep && sb_start)
        sb_end->setValue(sb_start->value());
}

// ---------------------------------------------------------------------------
void ParamConfigDialog::onFilter(const QString& text)
{
    for (int row = 0; row < table_->rowCount(); ++row) {
        auto* item = table_->item(row, COL_NAME);
        const bool visible = text.isEmpty() ||
            (item && item->text().contains(text, Qt::CaseInsensitive));
        table_->setRowHidden(row, !visible);
    }
}

// ---------------------------------------------------------------------------
std::vector<ParamDef> ParamConfigDialog::getParams() const
{
    std::vector<ParamDef> result;
    const std::string node_name = fetched_node_.isEmpty()
        ? node_edit_->text().trimmed().toStdString()
        : fetched_node_.toStdString();

    for (int row = 0; row < table_->rowCount(); ++row) {
        auto* inc = table_->item(row, COL_INCLUDE);
        if (!inc || inc->checkState() != Qt::Checked) continue;

        auto* name_item = table_->item(row, COL_NAME);
        auto* mode_cb   = qobject_cast<QComboBox*>(table_->cellWidget(row, COL_MODE));
        auto* sb_start  = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_START));
        auto* sb_end    = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_END));
        auto* sb_step   = qobject_cast<QDoubleSpinBox*>(table_->cellWidget(row, COL_STEP));
        if (!name_item || !mode_cb || !sb_start || !sb_end || !sb_step) continue;

        ParamDef def;
        def.node_name  = node_name;
        def.param_name = name_item->text().toStdString();
        def.start      = sb_start->value();

        if (mode_cb->currentIndex() == 0) {   // Fixed
            def.end  = def.start;
            def.step = 1.0;
        } else {                               // Sweep
            def.end  = sb_end->value();
            def.step = std::max(sb_step->value(), 1e-9);
        }
        result.push_back(def);
    }
    return result;
}
