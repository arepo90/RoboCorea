#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <rescue_interfaces/srv/go_to_pose.hpp>
#include <rescue_interfaces/srv/list_poses.hpp>

// Block-programmed autonomous arm routines ("like a macro editor"). An operator
// chains blocks into a named routine and the robot runs it on its own:
//
//   • Go to pose <name>        — /servo_node/go_to_pose; advances on plan_state "reached"
//   • Wait <seconds>           — dwell
//   • Gripper open/close <sec> — publishes /gripper rate (+open/−close) for N s
//   • Loop ×N { … }            — repeat the enclosed blocks N times (0 = until Stop)
//   • Run subroutine <routine> — call another saved routine (cycle/depth guarded)
//
// The interpreter is a Qt-thread state machine with a call stack (subroutines)
// and per-frame loop counters. Start/Pause/Stop drive it; Pause holds at the
// current block (an in-flight move finishes, then it waits for Resume); Stop ends
// sequencing without yanking the arm (E-STOP is the hard stop). A top-level
// "loop" checkbox repeats the whole routine.
//
// Routines persist (multiple, named) to ~/.config/robocorea_gui/arm_routines.json.
// Pose names come from /servo_node/list_poses (with the saved_poses.json cache as
// a launch-time fallback, same set the digital-twin panel uses).
class RoutineProgrammer : public QWidget {
    Q_OBJECT
public:
    explicit RoutineProgrammer(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    bool isRunning() const { return running_; }

signals:
    // ROS thread → Qt thread.
    void goResult(bool ok, const QString& message);
    void planStateChanged(const QString& text);
    void posesRefreshed(const QStringList& names);
    void armStateChanged(const QString& state);

private slots:
    // Routine library (auto-saves to disk on every edit).
    void onNewRoutine();
    void onRenameRoutine();
    void onDeleteRoutine();
    void onRoutineSelected(int index);
    void onLoopToggled(bool checked);
    // Block editing.
    void onAddPose();
    void onAddWait();
    void onAddGripper();
    void onAddLoopBegin();
    void onAddLoopEnd();
    void onAddSubroutine();
    void onRemoveBlock();
    void onMoveBlockUp();
    void onMoveBlockDown();
    void onRefreshPoses();
    // Run controls.
    void onStart();
    void onPauseToggled(bool checked);
    void onStop();
    // ROS-driven.
    void onGoResult(bool ok, const QString& message);
    void onPlanState(const QString& text);
    void onPosesRefreshed(const QStringList& names);
    void onArmState(const QString& state);
    void onTick();   // ~100 ms: Wait/Gripper dwell + gripper rate refresh

private:
    enum class BlockType { GoToPose, Wait, Gripper, LoopBegin, LoopEnd, RunSub };
    struct Block {
        BlockType type{BlockType::Wait};
        QString   str;          // GoToPose: pose; RunSub: routine; Gripper: "open"/"close"
        double    num{1.0};     // Wait/Gripper: seconds; LoopBegin: count (0 = until Stop)
    };
    struct Routine {
        QString        name;
        bool           loop{false};
        QVector<Block> blocks;
    };
    // Interpreter frames.
    struct LoopFrame { int begin_index{0}; int remaining{0}; bool infinite{false}; };
    struct CallFrame { int routine_idx{-1}; int pc{0}; QVector<LoopFrame> loops; };
    enum class Exec { Idle, Pose, Dwell, Gripper };

    // ── engine ────────────────────────────────────────────────────────────────
    void runNext();              // dispatch blocks until a busy one (or finish)
    void blockCompleted();       // a busy block finished → continue (honors Pause)
    void finishRoutine(const QString& reason, const QString& color);
    void publishGripper(double rate);
    void setRunUiState();

    // ── routine library ─────────────────────────────────────────────────────
    Routine* currentRoutine();
    int      routineIndexByName(const QString& name) const;
    void     rebuildBlockList();         // routine.blocks → list rows (indented)
    void     highlightBlock(int idx);    // mark the executing row (-1 clears)
    void     insertBlock(const Block& b);
    QString  blockText(const Block& b) const;
    void     rebuildRoutineCombo(const QString& select);
    void     rebuildSubroutineCombo();

    // ── persistence (~/.config/robocorea_gui/arm_routines.json) ───────────────
    QString routinesPath() const;
    void    loadRoutines();
    void    saveRoutines() const;

    // ── poses ────────────────────────────────────────────────────────────────
    void        populatePoseCombo(const QStringList& names);
    QStringList loadPoseCache() const;

    void setStatus(const QString& text, const QString& color = "#888");

    rclcpp::Node::SharedPtr node_;

    // Library.
    QVector<Routine> routines_;
    int              current_idx_{-1};
    QStringList      pose_names_;

    // Engine state.
    bool             running_{false};
    bool             paused_{false};
    Exec             exec_{Exec::Idle};
    QVector<CallFrame> call_stack_;
    bool             awaiting_reached_{false};
    bool             pending_run_{false};      // block finished while paused
    double           dwell_remaining_{0.0};    // Wait/Gripper seconds left
    double           gripper_rate_{0.0};       // active gripper rate (+open/−close)
    int              exec_routine_idx_{-1};    // for highlighting the live block
    int              exec_block_{-1};
    QString          arm_state_{"UNINIT"};

    // Widgets — library.
    QComboBox*      routine_combo_{nullptr};
    QPushButton*    new_btn_{nullptr};
    QPushButton*    rename_btn_{nullptr};
    QPushButton*    delete_btn_{nullptr};
    QCheckBox*      loop_check_{nullptr};
    // Widgets — block list + editing.
    QListWidget*    block_list_{nullptr};
    QComboBox*      pose_combo_{nullptr};
    QPushButton*    add_pose_btn_{nullptr};
    QPushButton*    refresh_poses_btn_{nullptr};
    QDoubleSpinBox* wait_spin_{nullptr};
    QPushButton*    add_wait_btn_{nullptr};
    QComboBox*      gripper_dir_{nullptr};
    QDoubleSpinBox* gripper_spin_{nullptr};
    QPushButton*    add_gripper_btn_{nullptr};
    QSpinBox*       loop_count_{nullptr};
    QPushButton*    add_loop_btn_{nullptr};
    QPushButton*    add_endloop_btn_{nullptr};
    QComboBox*      sub_combo_{nullptr};
    QPushButton*    add_sub_btn_{nullptr};
    QPushButton*    remove_btn_{nullptr};
    QPushButton*    up_btn_{nullptr};
    QPushButton*    down_btn_{nullptr};
    // Widgets — run.
    QPushButton*    start_btn_{nullptr};
    QPushButton*    pause_btn_{nullptr};
    QPushButton*    stop_btn_{nullptr};
    QLabel*         arm_state_label_{nullptr};
    QLabel*         status_label_{nullptr};

    QTimer*         tick_timer_{nullptr};

    // ROS.
    rclcpp::Client<rescue_interfaces::srv::GoToPose>::SharedPtr   go_cli_;
    rclcpp::Client<rescue_interfaces::srv::ListPoses>::SharedPtr  list_cli_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        plan_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        arm_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          gripper_pub_;
};
