#include "simulator/gui/main_window.hpp"
#include "simulator/gui/map_view.hpp"
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QToolBar>
#include <QAction>
#include <QStatusBar>
#include <QSplitter>
#include <QScrollArea>
#include <QMenu>
#include <QMenuBar>
#include <QComboBox>
#include <QString>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Helpers: simple parameter dialog builders
// ---------------------------------------------------------------------------
namespace {

// Returns false if cancelled
bool dialogStaticCircle(QWidget* parent, double& radius) {
    QDialog dlg(parent);
    dlg.setWindowTitle("Add Static Circle");
    auto* form  = new QFormLayout;
    auto* r_box = new QDoubleSpinBox;
    r_box->setRange(0.05, 20.0);
    r_box->setValue(0.3);
    r_box->setSuffix(" m");
    r_box->setSingleStep(0.05);
    form->addRow("Radius:", r_box);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout;
    layout->addLayout(form);
    layout->addWidget(btns);
    dlg.setLayout(layout);
    if (dlg.exec() != QDialog::Accepted) return false;
    radius = r_box->value();
    return true;
}

bool dialogStaticRect(QWidget* parent, double& w, double& h, double& angle_deg) {
    QDialog dlg(parent);
    dlg.setWindowTitle("Add Static Rectangle");
    auto* form  = new QFormLayout;
    auto* w_box = new QDoubleSpinBox; w_box->setRange(0.05, 20.0); w_box->setValue(0.5); w_box->setSuffix(" m");
    auto* h_box = new QDoubleSpinBox; h_box->setRange(0.05, 20.0); h_box->setValue(0.5); h_box->setSuffix(" m");
    auto* a_box = new QDoubleSpinBox; a_box->setRange(-180, 180);  a_box->setValue(0);   a_box->setSuffix(" °");
    form->addRow("Width:",  w_box);
    form->addRow("Height:", h_box);
    form->addRow("Angle:",  a_box);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout;
    layout->addLayout(form);
    layout->addWidget(btns);
    dlg.setLayout(layout);
    if (dlg.exec() != QDialog::Accepted) return false;
    w = w_box->value(); h = h_box->value(); angle_deg = a_box->value();
    return true;
}

bool dialogDynamic(QWidget* parent,
                   ObstacleShape& shape, double& radius,
                   double& width, double& height, double& speed) {
    QDialog dlg(parent);
    dlg.setWindowTitle("Add Dynamic Obstacle");
    auto* form      = new QFormLayout;
    auto* shape_box = new QComboBox;
    shape_box->addItem("Circle");
    shape_box->addItem("Rectangle");
    auto* r_box  = new QDoubleSpinBox; r_box->setRange(0.05, 10.0); r_box->setValue(0.3); r_box->setSuffix(" m");
    auto* w_box  = new QDoubleSpinBox; w_box->setRange(0.05, 10.0); w_box->setValue(0.5); w_box->setSuffix(" m");
    auto* h_box  = new QDoubleSpinBox; h_box->setRange(0.05, 10.0); h_box->setValue(0.5); h_box->setSuffix(" m");
    auto* sp_box = new QDoubleSpinBox; sp_box->setRange(0.01, 10.0); sp_box->setValue(0.5); sp_box->setSuffix(" m/s");
    form->addRow("Shape:",  shape_box);
    form->addRow("Radius:", r_box);
    form->addRow("Width:",  w_box);
    form->addRow("Height:", h_box);
    form->addRow("Speed:",  sp_box);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout;
    layout->addLayout(form);
    layout->addWidget(btns);
    dlg.setLayout(layout);
    if (dlg.exec() != QDialog::Accepted) return false;
    shape  = (shape_box->currentIndex() == 0) ? ObstacleShape::CIRCLE
                                               : ObstacleShape::RECTANGLE;
    radius = r_box->value();
    width  = w_box->value();
    height = h_box->value();
    speed  = sp_box->value();
    return true;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
MainWindow::MainWindow(std::shared_ptr<SimulatorNode> node, QWidget* parent)
: QMainWindow(parent), node_(node)
{
    setWindowTitle("ROS2 Simulator");
    resize(1280, 800);

    setupToolbar();

    // Central widget: map view
    map_view_ = new MapView(this);
    setCentralWidget(map_view_);

    setupDockPanel();

    // Status bar
    status_label_ = new QLabel("No map loaded.", this);
    statusBar()->addPermanentWidget(status_label_, 1);

    // Connect MapView signals
    connect(map_view_, &MapView::requestAddStaticCircle,
            this, &MainWindow::onAddStaticCircle);
    connect(map_view_, &MapView::requestAddStaticRect,
            this, &MainWindow::onAddStaticRect);
    connect(map_view_, &MapView::requestAddDynamic,
            this, &MainWindow::onAddDynamic);
    connect(map_view_, &MapView::requestDelete,
            this, &MainWindow::onDeleteObstacle);
    connect(map_view_, &MapView::requestSetPose,
            this, &MainWindow::onSetPose);
    connect(map_view_, &MapView::obstacleClicked,
            this, &MainWindow::onObstacleClicked);

    // Path drawing: update node + re-publish whenever waypoints change
    connect(map_view_, &MapView::pathWaypointsChanged,
            this, [this](const std::vector<std::pair<double,double>>& wps) {
                if (wps.empty()) {
                    node_->clearPath();
                } else if (node_->hasMap()) {
                    double res = node_->getMap().resolution;
                    node_->setPath(wps, res);
                }
                // Mirror to sweep panel waypoint table
                if (sweep_panel_) sweep_panel_->setPathWaypoints(wps);
            });

    // Physics timer at ~50 Hz
    phys_timer_ = new QTimer(this);
    connect(phys_timer_, &QTimer::timeout, this, &MainWindow::onPhysicsUpdate);
    phys_timer_->start(20);  // 20 ms = 50 Hz
}

// ---------------------------------------------------------------------------
void MainWindow::setupToolbar() {
    // ---- File toolbar -------------------------------------------------------
    auto* file_tb = addToolBar("File");
    file_tb->setMovable(false);

    auto* act_open_map  = file_tb->addAction(QIcon::fromTheme("document-open"),
                                              "Open Map (.yaml)");
    auto* act_load_urdf = file_tb->addAction(QIcon::fromTheme("system-run"),
                                              "Load Robot URDF");
    connect(act_open_map,  &QAction::triggered, this, &MainWindow::onOpenMap);
    connect(act_load_urdf, &QAction::triggered, this, &MainWindow::onLoadURDF);

    file_tb->addSeparator();

    // Speed slider
    speed_label_ = new QLabel("  Speed: 1×  ", this);
    file_tb->addWidget(speed_label_);
    auto* slider = new QSlider(Qt::Horizontal, this);
    slider->setRange(1, 20);   // 1× … 20×
    slider->setValue(1);
    slider->setFixedWidth(120);
    slider->setToolTip("Simulation speed multiplier");
    file_tb->addWidget(slider);
    connect(slider, &QSlider::valueChanged, this, &MainWindow::onSpeedChanged);

    // ---- Edit toolbar -------------------------------------------------------
    auto* edit_tb = addToolBar("Edit");
    edit_tb->setMovable(false);

    tool_group_ = new QActionGroup(this);
    tool_group_->setExclusive(true);

    auto makeToolAction = [&](const QString& text, EditTool tool) {
        auto* a = edit_tb->addAction(text);
        a->setCheckable(true);
        a->setData(static_cast<int>(tool));
        tool_group_->addAction(a);
        connect(a, &QAction::triggered, this, [this, tool]() {
            map_view_->setTool(tool);
        });
        return a;
    };

    auto* a_select = makeToolAction("Select",         EditTool::SELECT);
    makeToolAction("+Circle (S)",  EditTool::ADD_STATIC_CIRCLE);
    makeToolAction("+Rect (S)",    EditTool::ADD_STATIC_RECT);
    makeToolAction("+Dynamic",     EditTool::ADD_DYNAMIC);
    makeToolAction("Delete",       EditTool::DELETE_OBSTACLE);
    makeToolAction("Set Pose",     EditTool::SET_ROBOT_POSE);
    makeToolAction("Draw Path",    EditTool::DRAW_PATH);

    edit_tb->addSeparator();
    auto* act_clear_path = edit_tb->addAction("Clear Path");
    act_clear_path->setToolTip("Remove all path waypoints");
    connect(act_clear_path, &QAction::triggered, this, [this]() {
        map_view_->clearPath();
        node_->clearPath();
    });

    a_select->setChecked(true);

    // ---- map→odom TF toggle (separate toolbar) ------------------------------
    auto* tf_tb = addToolBar("TF");
    tf_tb->setMovable(false);

    auto* act_map_odom = tf_tb->addAction("map→odom TF");
    act_map_odom->setCheckable(true);
    act_map_odom->setChecked(false);
    act_map_odom->setToolTip(
        "Publish identity map→odom TF.\n"
        "Enable when NOT using a localization node (e.g. AMCL).");
    connect(act_map_odom, &QAction::toggled, this, [this](bool checked) {
        node_->publish_map_odom_tf.store(checked);
    });
}

// ---------------------------------------------------------------------------
void MainWindow::setupDockPanel() {
    auto* dock  = new QDockWidget("Obstacles", this);
    dock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    dock->setMinimumWidth(240);

    auto* container = new QWidget;
    auto* vbox      = new QVBoxLayout(container);

    // Obstacle list
    obs_list_ = new QListWidget;
    obs_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(obs_list_, &QListWidget::itemClicked,
            this, &MainWindow::onObstacleListClicked);
    vbox->addWidget(new QLabel("Obstacle list:"));
    vbox->addWidget(obs_list_, 2);

    auto* btn_remove = new QPushButton("Remove Selected");
    connect(btn_remove, &QPushButton::clicked, this, &MainWindow::onRemoveObstacle);
    vbox->addWidget(btn_remove);

    vbox->addSpacing(8);

    // ---- Properties group ---------------------------------------------------
    auto* grp    = new QGroupBox("Properties");
    auto* f      = new QFormLayout(grp);

    props_title_ = new QLabel("(none selected)");
    f->addRow(props_title_);

    prop_x_     = new QDoubleSpinBox; prop_x_->setRange(-999, 999); prop_x_->setSuffix(" m"); prop_x_->setSingleStep(0.1);
    prop_y_     = new QDoubleSpinBox; prop_y_->setRange(-999, 999); prop_y_->setSuffix(" m"); prop_y_->setSingleStep(0.1);
    prop_r_     = new QDoubleSpinBox; prop_r_->setRange(0.05, 20);  prop_r_->setSuffix(" m"); prop_r_->setSingleStep(0.05);

    f->addRow("X:",      prop_x_);
    f->addRow("Y:",      prop_y_);
    f->addRow("Radius:", prop_r_);

    // Rectangle-only params (hidden by default)
    rect_params_ = new QWidget;
    auto* rf     = new QFormLayout(rect_params_);
    rf->setContentsMargins(0, 0, 0, 0);
    prop_w_     = new QDoubleSpinBox; prop_w_->setRange(0.05, 20); prop_w_->setSuffix(" m"); prop_w_->setSingleStep(0.05);
    prop_h_     = new QDoubleSpinBox; prop_h_->setRange(0.05, 20); prop_h_->setSuffix(" m"); prop_h_->setSingleStep(0.05);
    prop_angle_ = new QDoubleSpinBox; prop_angle_->setRange(-180, 180); prop_angle_->setSuffix(" °"); prop_angle_->setSingleStep(1);
    rf->addRow("Width:",  prop_w_);
    rf->addRow("Height:", prop_h_);
    rf->addRow("Angle:",  prop_angle_);
    rect_params_->hide();
    f->addRow(rect_params_);

    // Dynamic params (hidden by default)
    prop_dynamic_ = new QCheckBox("Dynamic");
    f->addRow(prop_dynamic_);

    dyn_params_ = new QWidget;
    auto* df    = new QFormLayout(dyn_params_);
    df->setContentsMargins(0, 0, 0, 0);
    prop_ex_    = new QDoubleSpinBox; prop_ex_->setRange(-999, 999); prop_ex_->setSuffix(" m"); prop_ex_->setSingleStep(0.1);
    prop_ey_    = new QDoubleSpinBox; prop_ey_->setRange(-999, 999); prop_ey_->setSuffix(" m"); prop_ey_->setSingleStep(0.1);
    prop_speed_ = new QDoubleSpinBox; prop_speed_->setRange(0.01, 10); prop_speed_->setSuffix(" m/s"); prop_speed_->setSingleStep(0.1);
    df->addRow("End X:", prop_ex_);
    df->addRow("End Y:", prop_ey_);
    df->addRow("Speed:", prop_speed_);
    dyn_params_->hide();
    f->addRow(dyn_params_);

    connect(prop_dynamic_, &QCheckBox::toggled, dyn_params_, &QWidget::setVisible);

    btn_apply_ = new QPushButton("Apply");
    btn_apply_->setEnabled(false);
    connect(btn_apply_, &QPushButton::clicked, this, &MainWindow::onApplyProperties);
    f->addRow(btn_apply_);

    vbox->addWidget(grp, 1);
    dock->setWidget(container);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    // ---- Parameter Sweep dock (bottom) -------------------------------------
    auto* sweep_dock = new QDockWidget("Parameter Sweep", this);
    sweep_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
    sweep_panel_ = new SweepPanel(node_, this);

    // Wrap in a scroll area so the dock can shrink freely regardless of
    // the panel's minimumSizeHint — this is what allows window maximize to work.
    auto* scroll = new QScrollArea;
    scroll->setWidget(sweep_panel_);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    sweep_dock->setWidget(scroll);
    addDockWidget(Qt::BottomDockWidgetArea, sweep_dock);
}

// ---------------------------------------------------------------------------
void MainWindow::onOpenMap() {
    QString path = QFileDialog::getOpenFileName(
        this, "Open Map YAML", QString(),
        "ROS2 Map (*.yaml *.yml)");
    if (path.isEmpty()) return;

    try {
        MapData map = loadMap(path.toStdString());
        node_->setMap(map);
        map_view_->setMap(map);
        status_label_->setText(
            QString::asprintf("Map: %s  (%d×%d px, %.3f m/px)",
                QFileInfo(path).fileName().toUtf8().constData(),
                map.width, map.height, map.resolution));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Map Load Error", e.what());
    }
}

void MainWindow::onLoadURDF() {
    QString path = QFileDialog::getOpenFileName(
        this, "Load Robot URDF / Xacro", QString(),
        "Robot Description (*.urdf *.xacro *.xml);;URDF (*.urdf *.xml);;Xacro (*.xacro)");
    if (path.isEmpty()) return;

    double radius = parseURDFRadius(path.toStdString());
    node_->setRobotRadius(radius);
    status_label_->setText(
        QString::asprintf("URDF loaded: %s  (radius=%.3f m)",
            QFileInfo(path).fileName().toUtf8().constData(), radius));
}

// ---------------------------------------------------------------------------
void MainWindow::onPhysicsUpdate() {
    constexpr double base_dt = 0.02;  // 50 Hz
    double sim_dt = base_dt * node_->sim_speed.load();
    node_->physicsStep(sim_dt);
    node_->publishState();

    if (sweep_panel_)
        sweep_panel_->tick(base_dt, sim_dt, node_->getRobotState());

    // Refresh view
    auto rs = node_->getRobotState();
    map_view_->updateRobot(rs.x, rs.y, rs.theta, rs.radius, rs.footprint);
    map_view_->syncObstacles(node_->getObstacles());

    status_label_->setText(
        QString::asprintf("x=%.2f  y=%.2f  θ=%.1f°  | speed=%d×",
            rs.x, rs.y, rs.theta * 180.0 / M_PI,
            static_cast<int>(node_->sim_speed.load())));
}

void MainWindow::onSpeedChanged(int value) {
    node_->sim_speed.store(static_cast<double>(value));
    speed_label_->setText(QString("  Speed: %1×  ").arg(value));
}

// ---------------------------------------------------------------------------
// Obstacle list management
// ---------------------------------------------------------------------------
void MainWindow::refreshObstacleList() {
    obs_list_->blockSignals(true);
    obs_list_->clear();
    for (const auto& obs : node_->getObstacles()) {
        QString label =
            QString("#%1 %2 (%3)")
                .arg(obs.id)
                .arg(obs.shape == ObstacleShape::CIRCLE ? "Circle" : "Rect")
                .arg(obs.dynamic ? "D" : "S");
        auto* item = new QListWidgetItem(label);
        item->setData(Qt::UserRole, obs.id);
        obs_list_->addItem(item);
    }
    obs_list_->blockSignals(false);
}

void MainWindow::onObstacleListClicked(QListWidgetItem* item) {
    int id = item->data(Qt::UserRole).toInt();
    onObstacleClicked(id);
}

void MainWindow::onObstacleClicked(int id) {
    selected_obs_id_ = id;
    map_view_->selectObstacle(id);
    populateProperties(id);
    // Sync list selection
    for (int i = 0; i < obs_list_->count(); ++i) {
        if (obs_list_->item(i)->data(Qt::UserRole).toInt() == id) {
            obs_list_->setCurrentRow(i);
            break;
        }
    }
}

void MainWindow::onRemoveObstacle() {
    if (selected_obs_id_ < 0) return;
    node_->removeObstacle(selected_obs_id_);
    selected_obs_id_ = -1;
    clearProperties();
    refreshObstacleList();
}

void MainWindow::populateProperties(int id) {
    // Find obstacle in node
    auto obstacles = node_->getObstacles();
    for (const auto& obs : obstacles) {
        if (obs.id != id) continue;
        props_title_->setText(QString("Obstacle #%1").arg(id));
        prop_x_->setValue(obs.x);
        prop_y_->setValue(obs.y);
        prop_r_->setValue(obs.radius);
        prop_w_->setValue(obs.width);
        prop_h_->setValue(obs.height);
        prop_angle_->setValue(obs.angle * 180.0 / M_PI);
        prop_dynamic_->setChecked(obs.dynamic);
        prop_ex_->setValue(obs.end_x);
        prop_ey_->setValue(obs.end_y);
        prop_speed_->setValue(obs.speed);

        bool is_rect = (obs.shape == ObstacleShape::RECTANGLE);
        prop_r_->setVisible(!is_rect);
        rect_params_->setVisible(is_rect);
        dyn_params_->setVisible(obs.dynamic);
        btn_apply_->setEnabled(true);
        return;
    }
}

void MainWindow::clearProperties() {
    props_title_->setText("(none selected)");
    btn_apply_->setEnabled(false);
}

void MainWindow::onApplyProperties() {
    if (selected_obs_id_ < 0) return;
    auto obstacles = node_->getObstacles();
    for (auto obs : obstacles) {
        if (obs.id != selected_obs_id_) continue;
        obs.x       = prop_x_->value();
        obs.y       = prop_y_->value();
        obs.radius  = prop_r_->value();
        obs.width   = prop_w_->value();
        obs.height  = prop_h_->value();
        obs.angle   = prop_angle_->value() * M_PI / 180.0;
        obs.dynamic = prop_dynamic_->isChecked();
        obs.end_x   = prop_ex_->value();
        obs.end_y   = prop_ey_->value();
        obs.speed   = prop_speed_->value();
        node_->updateObstacle(obs);
        return;
    }
}

// ---------------------------------------------------------------------------
// Slots for MapView signals
// ---------------------------------------------------------------------------
void MainWindow::onAddStaticCircle(double x, double y) {
    double radius = 0.3;
    if (!dialogStaticCircle(this, radius)) return;

    Obstacle obs;
    obs.shape  = ObstacleShape::CIRCLE;
    obs.x      = x; obs.y = y;
    obs.radius = radius;
    obs.dynamic = false;
    node_->addObstacle(obs);
    refreshObstacleList();
}

void MainWindow::onAddStaticRect(double x1, double y1, double x2, double y2) {
    // Centre + size derived from drag corners; allow override in dialog
    double cx = (x1 + x2) * 0.5, cy = (y1 + y2) * 0.5;
    double w  = std::abs(x2 - x1), h = std::abs(y2 - y1);
    double angle_deg = 0.0;
    if (!dialogStaticRect(this, w, h, angle_deg)) return;

    Obstacle obs;
    obs.shape   = ObstacleShape::RECTANGLE;
    obs.x       = cx; obs.y = cy;
    obs.width   = w;  obs.height = h;
    obs.angle   = angle_deg * M_PI / 180.0;
    obs.dynamic = false;
    node_->addObstacle(obs);
    refreshObstacleList();
}

void MainWindow::onAddDynamic(double sx, double sy, double ex, double ey) {
    ObstacleShape shape; double radius, width, height, speed;
    if (!dialogDynamic(this, shape, radius, width, height, speed)) return;

    Obstacle obs;
    obs.shape   = shape;
    obs.x       = sx; obs.y = sy;
    obs.radius  = radius;
    obs.width   = width; obs.height = height;
    obs.dynamic = true;
    obs.start_x = sx; obs.start_y = sy;
    obs.end_x   = ex; obs.end_y   = ey;
    obs.speed   = speed;
    obs.dir     = 1;
    node_->addObstacle(obs);
    refreshObstacleList();
}

void MainWindow::onDeleteObstacle(int id) {
    node_->removeObstacle(id);
    if (selected_obs_id_ == id) {
        selected_obs_id_ = -1;
        clearProperties();
    }
    refreshObstacleList();
}

void MainWindow::onSetPose(double x, double y, double theta) {
    node_->setRobotPose(x, y, theta);
}
