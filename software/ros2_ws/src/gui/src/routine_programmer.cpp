#include "gui/routine_programmer.hpp"

#include <QBrush>
#include <QColor>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace {
constexpr double kTickS = 0.1;          // 100 ms engine tick
constexpr int    kTickMs = 100;
constexpr double kGripperRate = 1.0;    // /gripper rate magnitude (+open/−close)
constexpr int    kMaxCallDepth = 16;    // subroutine nesting cap
constexpr int    kRunGuard = 100000;    // tight-loop safety cap per runNext()

const QString kBlue  = "#4fc3f7";
const QString kGreen = "#8afa8a";
const QString kRed   = "#cc6666";
const QString kAmber = "#ccaa00";
const QString kGrey  = "#888";

QString btnStyle(const char* bg, const char* hover, const char* pressed)
{
    return QString(
        "QPushButton { background-color: %1; color: white; padding: 5px; "
        "border: 1px solid %2; border-radius: 3px; }"
        "QPushButton:hover { background-color: %2; }"
        "QPushButton:pressed { background-color: %3; }"
        "QPushButton:disabled { background-color: #2a2a35; color: #666; }")
        .arg(bg, hover, pressed);
}
}  // namespace

RoutineProgrammer::RoutineProgrammer(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    // ── Routine library ───────────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("Routine", this);
        auto* v = new QVBoxLayout(box);
        auto* row = new QHBoxLayout();
        routine_combo_ = new QComboBox(box);
        routine_combo_->setMinimumWidth(140);
        connect(routine_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
                this, &RoutineProgrammer::onRoutineSelected);
        new_btn_ = new QPushButton("New", box);
        new_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        connect(new_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onNewRoutine);
        rename_btn_ = new QPushButton("Rename", box);
        rename_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
        connect(rename_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onRenameRoutine);
        delete_btn_ = new QPushButton("Del", box);
        delete_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(delete_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onDeleteRoutine);
        row->addWidget(routine_combo_, 1);
        row->addWidget(new_btn_);
        row->addWidget(rename_btn_);
        row->addWidget(delete_btn_);
        v->addLayout(row);

        loop_check_ = new QCheckBox("Loop the whole routine until Stop", box);
        loop_check_->setStyleSheet("color: #bbb;");
        connect(loop_check_, &QCheckBox::toggled, this, &RoutineProgrammer::onLoopToggled);
        v->addWidget(loop_check_);
        root->addWidget(box);
    }

    // ── Blocks ──────────────────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("Blocks", this);
        auto* v = new QVBoxLayout(box);
        block_list_ = new QListWidget(box);
        block_list_->setMinimumHeight(160);
        block_list_->setStyleSheet(
            "QListWidget { background-color: #14141f; color: #c8c8c8; "
            "border: 1px solid #333; font-family: monospace; }");
        v->addWidget(block_list_);

        auto* row = new QHBoxLayout();
        remove_btn_ = new QPushButton("Remove", box);
        remove_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(remove_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onRemoveBlock);
        up_btn_ = new QPushButton("↑", box);
        up_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
        connect(up_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onMoveBlockUp);
        down_btn_ = new QPushButton("↓", box);
        down_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
        connect(down_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onMoveBlockDown);
        row->addWidget(remove_btn_);
        row->addStretch();
        row->addWidget(up_btn_);
        row->addWidget(down_btn_);
        v->addLayout(row);
        root->addWidget(box);
    }

    // ── Add block ─────────────────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("Add block", this);
        auto* v = new QVBoxLayout(box);
        const QString add_style = btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a");

        auto* pr = new QHBoxLayout();
        pose_combo_ = new QComboBox(box);
        pose_combo_->setMinimumWidth(120);
        refresh_poses_btn_ = new QPushButton("⟳", box);
        refresh_poses_btn_->setFixedWidth(28);
        refresh_poses_btn_->setToolTip("Refresh saved poses from the servo");
        refresh_poses_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
        connect(refresh_poses_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onRefreshPoses);
        add_pose_btn_ = new QPushButton("Add Go-to-pose", box);
        add_pose_btn_->setStyleSheet(add_style);
        connect(add_pose_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddPose);
        pr->addWidget(pose_combo_, 1);
        pr->addWidget(refresh_poses_btn_);
        pr->addWidget(add_pose_btn_);
        v->addLayout(pr);

        auto* wr = new QHBoxLayout();
        wait_spin_ = new QDoubleSpinBox(box);
        wait_spin_->setRange(0.0, 600.0);
        wait_spin_->setDecimals(1);
        wait_spin_->setSingleStep(0.5);
        wait_spin_->setValue(2.0);
        wait_spin_->setSuffix(" s");
        add_wait_btn_ = new QPushButton("Add Wait", box);
        add_wait_btn_->setStyleSheet(add_style);
        connect(add_wait_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddWait);
        wr->addWidget(wait_spin_, 1);
        wr->addWidget(add_wait_btn_);
        v->addLayout(wr);

        auto* gr = new QHBoxLayout();
        gripper_dir_ = new QComboBox(box);
        gripper_dir_->addItems({"Open", "Close"});
        gripper_spin_ = new QDoubleSpinBox(box);
        gripper_spin_->setRange(0.1, 30.0);
        gripper_spin_->setDecimals(1);
        gripper_spin_->setSingleStep(0.5);
        gripper_spin_->setValue(1.5);
        gripper_spin_->setSuffix(" s");
        add_gripper_btn_ = new QPushButton("Add Gripper", box);
        add_gripper_btn_->setStyleSheet(add_style);
        connect(add_gripper_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddGripper);
        gr->addWidget(gripper_dir_);
        gr->addWidget(gripper_spin_, 1);
        gr->addWidget(add_gripper_btn_);
        v->addLayout(gr);

        auto* lr = new QHBoxLayout();
        loop_count_ = new QSpinBox(box);
        loop_count_->setRange(0, 9999);
        loop_count_->setValue(2);
        loop_count_->setPrefix("×");
        loop_count_->setToolTip("Repeat count (0 = repeat until Stop)");
        add_loop_btn_ = new QPushButton("Add Loop {", box);
        add_loop_btn_->setStyleSheet(add_style);
        connect(add_loop_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddLoopBegin);
        add_endloop_btn_ = new QPushButton("Add } End Loop", box);
        add_endloop_btn_->setStyleSheet(add_style);
        connect(add_endloop_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddLoopEnd);
        lr->addWidget(loop_count_);
        lr->addWidget(add_loop_btn_);
        lr->addWidget(add_endloop_btn_);
        v->addLayout(lr);

        auto* sr = new QHBoxLayout();
        sub_combo_ = new QComboBox(box);
        sub_combo_->setMinimumWidth(120);
        add_sub_btn_ = new QPushButton("Add Subroutine", box);
        add_sub_btn_->setStyleSheet(add_style);
        connect(add_sub_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onAddSubroutine);
        sr->addWidget(sub_combo_, 1);
        sr->addWidget(add_sub_btn_);
        v->addLayout(sr);
        root->addWidget(box);
    }

    // ── Run ───────────────────────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("Run", this);
        auto* v = new QVBoxLayout(box);
        arm_state_label_ = new QLabel("Arm: —", box);
        arm_state_label_->setStyleSheet("color: #888;");
        v->addWidget(arm_state_label_);

        auto* row = new QHBoxLayout();
        start_btn_ = new QPushButton("▶ Start", box);
        start_btn_->setMinimumHeight(34);
        start_btn_->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(start_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onStart);
        pause_btn_ = new QPushButton("⏸ Pause", box);
        pause_btn_->setMinimumHeight(34);
        pause_btn_->setCheckable(true);
        pause_btn_->setStyleSheet(
            "QPushButton { background-color: #4a4a2a; color: white; padding: 5px; "
            "border: 1px solid #6a6a3a; border-radius: 3px; }"
            "QPushButton:hover { background-color: #6a6a3a; }"
            "QPushButton:checked { background-color: #c8870a; color: black; border-color: #ffb13a; }"
            "QPushButton:disabled { background-color: #2a2a35; color: #666; }");
        connect(pause_btn_, &QPushButton::toggled, this, &RoutineProgrammer::onPauseToggled);
        stop_btn_ = new QPushButton("⏹ Stop", box);
        stop_btn_->setMinimumHeight(34);
        stop_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(stop_btn_, &QPushButton::clicked, this, &RoutineProgrammer::onStop);
        row->addWidget(start_btn_);
        row->addWidget(pause_btn_);
        row->addWidget(stop_btn_);
        v->addLayout(row);

        status_label_ = new QLabel("—", box);
        status_label_->setWordWrap(true);
        status_label_->setStyleSheet("color: #888; font-size: 12px;");
        v->addWidget(status_label_);
        root->addWidget(box);
    }

    // ── ROS ─────────────────────────────────────────────────────────────────
    go_cli_   = node_->create_client<rescue_interfaces::srv::GoToPose>("/servo_node/go_to_pose");
    list_cli_ = node_->create_client<rescue_interfaces::srv::ListPoses>("/servo_node/list_poses");
    gripper_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/gripper", rclcpp::QoS(10));

    plan_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/servo_node/plan_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr m) {
            emit planStateChanged(QString::fromStdString(m->data));
        });
    arm_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr m) {
            emit armStateChanged(QString::fromStdString(m->data));
        });

    connect(this, &RoutineProgrammer::goResult, this, &RoutineProgrammer::onGoResult,
            Qt::QueuedConnection);
    connect(this, &RoutineProgrammer::planStateChanged, this, &RoutineProgrammer::onPlanState,
            Qt::QueuedConnection);
    connect(this, &RoutineProgrammer::posesRefreshed, this, &RoutineProgrammer::onPosesRefreshed,
            Qt::QueuedConnection);
    connect(this, &RoutineProgrammer::armStateChanged, this, &RoutineProgrammer::onArmState,
            Qt::QueuedConnection);

    tick_timer_ = new QTimer(this);
    tick_timer_->setInterval(kTickMs);
    connect(tick_timer_, &QTimer::timeout, this, &RoutineProgrammer::onTick);
    tick_timer_->start();

    loadRoutines();
    populatePoseCombo(loadPoseCache());
    onRefreshPoses();   // reconcile with the servo when it is available
    setRunUiState();
}

// ── status / poses ─────────────────────────────────────────────────────────────

void RoutineProgrammer::setStatus(const QString& text, const QString& color)
{
    status_label_->setText(text);
    status_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));
}

QStringList RoutineProgrammer::loadPoseCache() const
{
    QFile f(QDir::homePath() + "/.config/robocorea_gui/saved_poses.json");
    if (!f.open(QIODevice::ReadOnly)) return {};
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return {};
    QStringList names;
    for (const QJsonValue& v : doc.object().value("poses").toArray()) {
        QString n = v.toString().trimmed();
        if (!n.isEmpty() && !names.contains(n)) names << n;
    }
    names.sort(Qt::CaseInsensitive);
    return names;
}

void RoutineProgrammer::onRefreshPoses()
{
    if (!list_cli_->service_is_ready()) return;   // keep cache; servo not up yet
    list_cli_->async_send_request(
        std::make_shared<rescue_interfaces::srv::ListPoses::Request>(),
        [this](rclcpp::Client<rescue_interfaces::srv::ListPoses>::SharedFuture f) {
            auto r = f.get();
            QStringList names;
            for (const auto& n : r->names) names << QString::fromStdString(n);
            emit posesRefreshed(names);
        });
}

void RoutineProgrammer::onPosesRefreshed(const QStringList& names)
{
    populatePoseCombo(names);
}

void RoutineProgrammer::populatePoseCombo(const QStringList& names)
{
    pose_names_ = names;
    const QString keep = pose_combo_->currentText();
    QSignalBlocker b(pose_combo_);
    pose_combo_->clear();
    pose_combo_->addItems(names);
    int i = pose_combo_->findText(keep);
    if (i >= 0) pose_combo_->setCurrentIndex(i);
    rebuildBlockList();   // refresh unknown-pose flags
}

// ── routine library ────────────────────────────────────────────────────────────

RoutineProgrammer::Routine* RoutineProgrammer::currentRoutine()
{
    return (current_idx_ >= 0 && current_idx_ < routines_.size()) ? &routines_[current_idx_]
                                                                  : nullptr;
}

int RoutineProgrammer::routineIndexByName(const QString& name) const
{
    for (int i = 0; i < routines_.size(); ++i)
        if (routines_[i].name == name) return i;
    return -1;
}

void RoutineProgrammer::onNewRoutine()
{
    bool ok = false;
    QString name = QInputDialog::getText(this, "New routine", "Routine name:",
                                         QLineEdit::Normal, "", &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    if (routineIndexByName(name) >= 0) {
        QMessageBox::warning(this, "New routine", "A routine with that name already exists.");
        return;
    }
    routines_.push_back(Routine{name, false, {}});
    current_idx_ = routines_.size() - 1;
    rebuildRoutineCombo(name);
    rebuildSubroutineCombo();
    rebuildBlockList();
    { QSignalBlocker b(loop_check_); loop_check_->setChecked(false); }
    saveRoutines();
    setRunUiState();
}

void RoutineProgrammer::onRenameRoutine()
{
    Routine* r = currentRoutine();
    if (!r) return;
    bool ok = false;
    QString name = QInputDialog::getText(this, "Rename routine", "New name:",
                                         QLineEdit::Normal, r->name, &ok).trimmed();
    if (!ok || name.isEmpty() || name == r->name) return;
    if (routineIndexByName(name) >= 0) {
        QMessageBox::warning(this, "Rename routine", "A routine with that name already exists.");
        return;
    }
    r->name = name;
    rebuildRoutineCombo(name);
    rebuildSubroutineCombo();
    saveRoutines();
}

void RoutineProgrammer::onDeleteRoutine()
{
    Routine* r = currentRoutine();
    if (!r) return;
    if (QMessageBox::question(this, "Delete routine",
                              QString("Delete routine '%1'?").arg(r->name))
        != QMessageBox::Yes)
        return;
    const QString gone = r->name;
    routines_.removeAt(current_idx_);
    current_idx_ = routines_.isEmpty() ? -1 : qMin(current_idx_, routines_.size() - 1);
    rebuildRoutineCombo(current_idx_ >= 0 ? routines_[current_idx_].name : QString());
    rebuildSubroutineCombo();
    rebuildBlockList();
    if (Routine* cur = currentRoutine()) {
        QSignalBlocker b(loop_check_);
        loop_check_->setChecked(cur->loop);
    }
    saveRoutines();
    setRunUiState();
}

void RoutineProgrammer::onRoutineSelected(int index)
{
    if (index < 0 || index >= routines_.size()) return;
    current_idx_ = index;
    { QSignalBlocker b(loop_check_); loop_check_->setChecked(routines_[index].loop); }
    rebuildBlockList();
    saveRoutines();   // persist 'selected'
    setRunUiState();
}

void RoutineProgrammer::onLoopToggled(bool checked)
{
    if (Routine* r = currentRoutine()) {
        r->loop = checked;
        saveRoutines();
    }
}

void RoutineProgrammer::rebuildRoutineCombo(const QString& select)
{
    QSignalBlocker b(routine_combo_);
    routine_combo_->clear();
    for (const Routine& r : routines_) routine_combo_->addItem(r.name);
    int i = routine_combo_->findText(select);
    if (i < 0) i = routines_.isEmpty() ? -1 : 0;
    routine_combo_->setCurrentIndex(i);
    current_idx_ = i;
}

void RoutineProgrammer::rebuildSubroutineCombo()
{
    QSignalBlocker b(sub_combo_);
    const QString keep = sub_combo_->currentText();
    sub_combo_->clear();
    for (const Routine& r : routines_) sub_combo_->addItem(r.name);
    int i = sub_combo_->findText(keep);
    if (i >= 0) sub_combo_->setCurrentIndex(i);
}

// ── block list ───────────────────────────────────────────────────────────────

QString RoutineProgrammer::blockText(const Block& b) const
{
    switch (b.type) {
        case BlockType::GoToPose: return QString("▸ Go to pose: %1").arg(b.str);
        case BlockType::Wait:     return QString("⏱ Wait %1 s").arg(b.num, 0, 'f', 1);
        case BlockType::Gripper:  return QString("✛ Gripper %1 %2 s")
                                      .arg(b.str == "open" ? "open" : "close")
                                      .arg(b.num, 0, 'f', 1);
        case BlockType::LoopBegin:
            return b.num <= 0 ? QString("⟳ Loop ×∞ {")
                              : QString("⟳ Loop ×%1 {").arg(static_cast<int>(b.num));
        case BlockType::LoopEnd:  return QString("}");
        case BlockType::RunSub:   return QString("⤷ Run subroutine: %1").arg(b.str);
    }
    return {};
}

void RoutineProgrammer::rebuildBlockList()
{
    block_list_->clear();
    Routine* r = currentRoutine();
    if (!r) return;
    int depth = 0;
    for (const Block& b : r->blocks) {
        int indent = (b.type == BlockType::LoopEnd) ? qMax(0, depth - 1) : depth;
        auto* it = new QListWidgetItem(QString(indent * 2, ' ') + blockText(b), block_list_);

        bool invalid = false;
        if (b.type == BlockType::GoToPose && !pose_names_.isEmpty()
            && !pose_names_.contains(b.str))
            invalid = true;   // pose not in the known set
        if (b.type == BlockType::RunSub
            && (routineIndexByName(b.str) < 0 || b.str == r->name))
            invalid = true;   // missing or self-referential subroutine
        if (b.type == BlockType::LoopEnd && depth == 0)
            invalid = true;   // stray end
        if (invalid) it->setForeground(QBrush(QColor("#cc6666")));

        if (b.type == BlockType::LoopBegin) depth++;
        if (b.type == BlockType::LoopEnd)   depth = qMax(0, depth - 1);
    }
    highlightBlock(exec_routine_idx_ == current_idx_ ? exec_block_ : -1);
}

void RoutineProgrammer::highlightBlock(int idx)
{
    for (int i = 0; i < block_list_->count(); ++i) {
        QListWidgetItem* it = block_list_->item(i);
        it->setBackground(i == idx ? QBrush(QColor("#2a4a2a")) : QBrush(Qt::NoBrush));
    }
    if (idx >= 0 && idx < block_list_->count())
        block_list_->scrollToItem(block_list_->item(idx));
}

void RoutineProgrammer::insertBlock(const Block& b)
{
    Routine* r = currentRoutine();
    if (!r) {
        setStatus("create a routine first", kAmber);
        return;
    }
    int pos = block_list_->currentRow();
    pos = (pos >= 0) ? pos + 1 : r->blocks.size();
    r->blocks.insert(pos, b);
    rebuildBlockList();
    block_list_->setCurrentRow(pos);
    saveRoutines();
    setRunUiState();
}

void RoutineProgrammer::onAddPose()
{
    QString name = pose_combo_->currentText().trimmed();
    if (name.isEmpty()) { setStatus("no pose selected", kAmber); return; }
    insertBlock(Block{BlockType::GoToPose, name, 0.0});
}

void RoutineProgrammer::onAddWait()
{
    insertBlock(Block{BlockType::Wait, QString(), wait_spin_->value()});
}

void RoutineProgrammer::onAddGripper()
{
    const QString dir = gripper_dir_->currentText() == "Open" ? "open" : "close";
    insertBlock(Block{BlockType::Gripper, dir, gripper_spin_->value()});
}

void RoutineProgrammer::onAddLoopBegin()
{
    insertBlock(Block{BlockType::LoopBegin, QString(), static_cast<double>(loop_count_->value())});
}

void RoutineProgrammer::onAddLoopEnd()
{
    insertBlock(Block{BlockType::LoopEnd, QString(), 0.0});
}

void RoutineProgrammer::onAddSubroutine()
{
    QString name = sub_combo_->currentText().trimmed();
    if (name.isEmpty()) { setStatus("no subroutine selected", kAmber); return; }
    insertBlock(Block{BlockType::RunSub, name, 0.0});
}

void RoutineProgrammer::onRemoveBlock()
{
    Routine* r = currentRoutine();
    int row = block_list_->currentRow();
    if (!r || row < 0 || row >= r->blocks.size()) return;
    r->blocks.removeAt(row);
    rebuildBlockList();
    block_list_->setCurrentRow(qMin(row, r->blocks.size() - 1));
    saveRoutines();
    setRunUiState();
}

void RoutineProgrammer::onMoveBlockUp()
{
    Routine* r = currentRoutine();
    int row = block_list_->currentRow();
    if (!r || row <= 0) return;
    r->blocks.move(row, row - 1);
    rebuildBlockList();
    block_list_->setCurrentRow(row - 1);
    saveRoutines();
}

void RoutineProgrammer::onMoveBlockDown()
{
    Routine* r = currentRoutine();
    int row = block_list_->currentRow();
    if (!r || row < 0 || row >= r->blocks.size() - 1) return;
    r->blocks.move(row, row + 1);
    rebuildBlockList();
    block_list_->setCurrentRow(row + 1);
    saveRoutines();
}

// ── persistence ────────────────────────────────────────────────────────────────

QString RoutineProgrammer::routinesPath() const
{
    return QDir::homePath() + "/.config/robocorea_gui/arm_routines.json";
}

void RoutineProgrammer::loadRoutines()
{
    QFile f(routinesPath());
    QString selected;
    if (f.open(QIODevice::ReadOnly)) {
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            const QJsonObject root = doc.object();
            selected = root.value("selected").toString();
            for (const QJsonValue& rv : root.value("routines").toArray()) {
                const QJsonObject ro = rv.toObject();
                Routine r;
                r.name = ro.value("name").toString();
                r.loop = ro.value("loop").toBool();
                if (r.name.isEmpty()) continue;
                for (const QJsonValue& bv : ro.value("blocks").toArray()) {
                    const QJsonObject bo = bv.toObject();
                    const QString t = bo.value("type").toString();
                    Block b;
                    b.str = bo.value("str").toString();
                    b.num = bo.value("num").toDouble();
                    if (t == "pose")            b.type = BlockType::GoToPose;
                    else if (t == "wait")       b.type = BlockType::Wait;
                    else if (t == "gripper")    b.type = BlockType::Gripper;
                    else if (t == "loop_begin") b.type = BlockType::LoopBegin;
                    else if (t == "loop_end")   b.type = BlockType::LoopEnd;
                    else if (t == "runsub")     b.type = BlockType::RunSub;
                    else continue;
                    r.blocks.push_back(b);
                }
                routines_.push_back(r);
            }
        }
    }
    rebuildRoutineCombo(selected);
    rebuildSubroutineCombo();
    if (Routine* r = currentRoutine()) {
        QSignalBlocker b(loop_check_);
        loop_check_->setChecked(r->loop);
    }
    rebuildBlockList();
}

void RoutineProgrammer::saveRoutines() const
{
    QJsonArray arr;
    for (const Routine& r : routines_) {
        QJsonObject ro;
        ro["name"] = r.name;
        ro["loop"] = r.loop;
        QJsonArray blocks;
        for (const Block& b : r.blocks) {
            QJsonObject bo;
            switch (b.type) {
                case BlockType::GoToPose:  bo["type"] = "pose";       bo["str"] = b.str; break;
                case BlockType::Wait:      bo["type"] = "wait";       bo["num"] = b.num; break;
                case BlockType::Gripper:   bo["type"] = "gripper";    bo["str"] = b.str;
                                           bo["num"] = b.num; break;
                case BlockType::LoopBegin: bo["type"] = "loop_begin"; bo["num"] = b.num; break;
                case BlockType::LoopEnd:   bo["type"] = "loop_end";   break;
                case BlockType::RunSub:    bo["type"] = "runsub";     bo["str"] = b.str; break;
            }
            blocks.append(bo);
        }
        ro["blocks"] = blocks;
        arr.append(ro);
    }
    QJsonObject root;
    root["version"] = 1;
    root["selected"] = (current_idx_ >= 0) ? routines_[current_idx_].name : QString();
    root["routines"] = arr;

    const QString path = routinesPath();
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

// ── engine ─────────────────────────────────────────────────────────────────────

void RoutineProgrammer::publishGripper(double rate)
{
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(rate);
    gripper_pub_->publish(msg);
}

void RoutineProgrammer::runNext()
{
    if (!running_) return;
    int guard = 0;
    while (true) {
        if (++guard > kRunGuard) {
            finishRoutine("aborted: no actionable steps (empty/tight loop?)", kRed);
            return;
        }
        if (call_stack_.isEmpty()) {
            Routine* top = currentRoutine();
            if (top && top->loop && !top->blocks.isEmpty()) {
                call_stack_.push_back(CallFrame{current_idx_, 0, {}});
                continue;
            }
            finishRoutine("completed", kGreen);
            return;
        }
        CallFrame& f = call_stack_.last();
        const Routine& r = routines_[f.routine_idx];
        if (f.pc >= r.blocks.size()) { call_stack_.removeLast(); continue; }

        const int idx = f.pc;
        const Block b = r.blocks[idx];
        switch (b.type) {
            case BlockType::GoToPose: {
                f.pc++;
                exec_routine_idx_ = f.routine_idx;
                exec_block_ = idx;
                rebuildBlockList();   // refresh highlight (matches displayed routine)
                if (!go_cli_->service_is_ready()) {
                    finishRoutine("aborted: servo go_to_pose unavailable", kRed);
                    return;
                }
                auto req = std::make_shared<rescue_interfaces::srv::GoToPose::Request>();
                req->name = b.str.toStdString();
                req->use_pose = false;
                exec_ = Exec::Pose;
                awaiting_reached_ = true;
                setStatus(QString("→ moving to '%1'…").arg(b.str), kBlue);
                go_cli_->async_send_request(
                    req, [this](rclcpp::Client<rescue_interfaces::srv::GoToPose>::SharedFuture fut) {
                        auto rr = fut.get();
                        emit goResult(rr->success, QString::fromStdString(rr->message));
                    });
                return;
            }
            case BlockType::Wait: {
                f.pc++;
                exec_routine_idx_ = f.routine_idx;
                exec_block_ = idx;
                rebuildBlockList();
                exec_ = Exec::Dwell;
                dwell_remaining_ = b.num;
                setStatus(QString("⏱ waiting %1 s…").arg(b.num, 0, 'f', 1), kBlue);
                return;
            }
            case BlockType::Gripper: {
                f.pc++;
                exec_routine_idx_ = f.routine_idx;
                exec_block_ = idx;
                rebuildBlockList();
                exec_ = Exec::Gripper;
                dwell_remaining_ = b.num;
                gripper_rate_ = (b.str == "open") ? kGripperRate : -kGripperRate;
                publishGripper(gripper_rate_);
                setStatus(QString("✛ gripper %1 %2 s…")
                              .arg(b.str == "open" ? "opening" : "closing")
                              .arg(b.num, 0, 'f', 1), kBlue);
                return;
            }
            case BlockType::LoopBegin: {
                f.pc++;
                const int count = static_cast<int>(b.num);
                f.loops.push_back(LoopFrame{idx, count, count <= 0});
                continue;
            }
            case BlockType::LoopEnd: {
                f.pc++;
                if (f.loops.isEmpty()) continue;   // stray end → ignore
                LoopFrame& lf = f.loops.last();
                if (lf.infinite) { f.pc = lf.begin_index + 1; continue; }
                if (--lf.remaining > 0) f.pc = lf.begin_index + 1;
                else                    f.loops.removeLast();
                continue;
            }
            case BlockType::RunSub: {
                f.pc++;
                const int sidx = routineIndexByName(b.str);
                if (sidx < 0) {
                    finishRoutine(QString("aborted: unknown subroutine '%1'").arg(b.str), kRed);
                    return;
                }
                bool on_stack = false;
                for (const CallFrame& cf : call_stack_)
                    if (cf.routine_idx == sidx) { on_stack = true; break; }
                if (on_stack) {
                    finishRoutine(QString("aborted: recursive subroutine '%1'").arg(b.str), kRed);
                    return;
                }
                if (call_stack_.size() >= kMaxCallDepth) {
                    finishRoutine("aborted: subroutine nesting too deep", kRed);
                    return;
                }
                call_stack_.push_back(CallFrame{sidx, 0, {}});
                continue;
            }
        }
    }
}

void RoutineProgrammer::blockCompleted()
{
    exec_ = Exec::Idle;
    awaiting_reached_ = false;
    if (!running_) return;
    if (paused_) {
        pending_run_ = true;
        setStatus("paused (step done)", kAmber);
        return;
    }
    runNext();
}

void RoutineProgrammer::finishRoutine(const QString& reason, const QString& color)
{
    running_ = false;
    paused_ = false;
    exec_ = Exec::Idle;
    awaiting_reached_ = false;
    pending_run_ = false;
    call_stack_.clear();
    dwell_remaining_ = 0.0;
    gripper_rate_ = 0.0;
    publishGripper(0.0);
    exec_routine_idx_ = -1;
    exec_block_ = -1;
    { QSignalBlocker b(pause_btn_); pause_btn_->setChecked(false); pause_btn_->setText("⏸ Pause"); }
    rebuildBlockList();
    setRunUiState();
    setStatus(reason, color);
}

void RoutineProgrammer::onStart()
{
    if (running_) return;
    Routine* r = currentRoutine();
    if (!r) { setStatus("no routine selected", kRed); return; }
    if (r->blocks.isEmpty()) { setStatus("routine is empty", kRed); return; }
    if (arm_state_ != "READY") {
        auto reply = QMessageBox::question(
            this, "Arm not READY",
            QString("The arm state is '%1', not READY — poses may fail to execute.\n\n"
                    "Start the routine anyway?").arg(arm_state_),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply != QMessageBox::Yes) return;
    }
    running_ = true;
    paused_ = false;
    pending_run_ = false;
    awaiting_reached_ = false;
    exec_ = Exec::Idle;
    call_stack_.clear();
    call_stack_.push_back(CallFrame{current_idx_, 0, {}});
    { QSignalBlocker b(pause_btn_); pause_btn_->setChecked(false); }
    setRunUiState();
    setStatus("running…", kBlue);
    runNext();
}

void RoutineProgrammer::onPauseToggled(bool checked)
{
    paused_ = checked;
    pause_btn_->setText(checked ? "▶ Resume" : "⏸ Pause");
    if (!running_) return;
    if (checked) {
        setStatus("paused", kAmber);
        if (exec_ == Exec::Gripper) publishGripper(0.0);   // stop gripper while held
    } else {
        setStatus("resumed", kBlue);
        if (pending_run_) { pending_run_ = false; runNext(); }
        // Dwell/Gripper resume on the next tick; a pose in flight resumes itself.
    }
}

void RoutineProgrammer::onStop()
{
    if (!running_) return;
    finishRoutine("stopped", kGrey);
}

void RoutineProgrammer::onTick()
{
    if (!running_) return;
    if (paused_) {
        if (exec_ == Exec::Gripper) publishGripper(0.0);
        return;
    }
    if (exec_ == Exec::Gripper) {
        publishGripper(gripper_rate_);   // keep fresh (firmware times out stale cmds)
        dwell_remaining_ -= kTickS;
        if (dwell_remaining_ <= 0.0) { publishGripper(0.0); blockCompleted(); }
        else setStatus(QString("✛ gripper %1 (%2 s)…")
                           .arg(gripper_rate_ > 0 ? "opening" : "closing")
                           .arg(dwell_remaining_, 0, 'f', 1), kBlue);
    } else if (exec_ == Exec::Dwell) {
        dwell_remaining_ -= kTickS;
        if (dwell_remaining_ <= 0.0) blockCompleted();
        else setStatus(QString("⏱ waiting %1 s…").arg(dwell_remaining_, 0, 'f', 1), kBlue);
    }
}

// ── ROS slots ──────────────────────────────────────────────────────────────────

void RoutineProgrammer::onGoResult(bool ok, const QString& message)
{
    if (!running_ || exec_ != Exec::Pose) return;   // stale
    if (!ok) finishRoutine("aborted: " + message, kRed);
    // ok: motion started; await plan_state "reached".
}

void RoutineProgrammer::onPlanState(const QString& text)
{
    if (!running_ || exec_ != Exec::Pose || !awaiting_reached_) return;
    if (text == "reached") {
        blockCompleted();
    } else if (text.startsWith("aborted") || text == "unreachable"
               || text.startsWith("plan failed")) {
        finishRoutine("aborted: arm " + text, kRed);
    }
    // planning / moving … → keep waiting
}

void RoutineProgrammer::onArmState(const QString& state)
{
    arm_state_ = state;
    QString color = "#888";
    if (state == "READY")             color = "#33cc33";
    else if (state == "INITIALIZING") color = "#ccaa00";
    else if (state == "FAULT")        color = "#cc3333";
    arm_state_label_->setText("Arm: " + state);
    arm_state_label_->setStyleSheet(QString("color: %1;").arg(color));
}

// ── enable/disable editing while running ────────────────────────────────────────

void RoutineProgrammer::setRunUiState()
{
    const bool editable = !running_;
    const std::initializer_list<QWidget*> editors = {
        routine_combo_, new_btn_, rename_btn_, delete_btn_, loop_check_, pose_combo_,
        refresh_poses_btn_, add_pose_btn_, wait_spin_, add_wait_btn_, gripper_dir_,
        gripper_spin_, add_gripper_btn_, loop_count_, add_loop_btn_, add_endloop_btn_,
        sub_combo_, add_sub_btn_, remove_btn_, up_btn_, down_btn_};
    for (QWidget* w : editors)
        w->setEnabled(editable);

    Routine* r = currentRoutine();
    start_btn_->setEnabled(editable && r && !r->blocks.isEmpty());
    pause_btn_->setEnabled(running_);
    stop_btn_->setEnabled(running_);
}
