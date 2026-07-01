#pragma once

#include <QDialog>
#include <string>

#include <rclcpp/rclcpp.hpp>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class PpmCalibDialog;

// Robot config dialog. Hosts the RC PPM calibration sub-dialog, plus audio/speech/
// thermal/detection-label preferences. RoboCorea is a single robot (Dicerox), so
// the legacy Jaguar/Dicerox robot-type switch is gone, and the RC now uses a fixed
// control scheme so there is no keybind editor (only PPM calibration remains).
//
// Apply mutates AppSettings live (and emits settingsApplied so MainWindow can
// republish ppm_calib + push grammar/audio); Save also persists to disk;
// Reset restores the values captured at open.
class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit SettingsDialog(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // Sync UI widgets with current AppSettings (call before show()).
    void reloadFromSettings();

signals:
    void settingsApplied();

private slots:
    void onApply();
    void onSave();
    void onCancel();
    void onPpmCalibClicked();

private:
    void applyToSettings();
    void captureOriginals();
    void restoreOriginals();

    // Snapshot taken at dialog open — used by Reset to undo any Apply
    bool        orig_audio_enabled_;
    bool        orig_talkback_enabled_;
    int         orig_colormap_, orig_interp_, orig_upscale_w_, orig_upscale_h_;
    int         orig_font_scale_x100_;
    std::string orig_grammar_;
    std::string orig_mic_device_;

    // Widgets
    QCheckBox* audio_check_;
    QCheckBox* talkback_check_;   // enable operator→robot talkback at startup
    QLineEdit* mic_device_edit_;  // talkback capture device (empty = default)
    QLineEdit* grammar_edit_;
    QComboBox* colormap_combo_;
    QComboBox* interp_combo_;
    QComboBox* upscale_combo_;
    QSlider*   font_scale_slider_;
    QLabel*    font_scale_val_;
    QPushButton* ppm_calib_btn_;

    rclcpp::Node::SharedPtr node_;

    PpmCalibDialog* ppm_calib_dialog_{nullptr};

    // Combo index ↔ cv enum value tables
    static const int COLORMAPS[5];
    static const int INTERPS[4];
    static const int UPSCALE_W[4];
    static const int UPSCALE_H[4];

    static int indexOfColormap(int cv_val);
    static int indexOfInterp(int cv_val);
    static int indexOfUpscale(int w);
};
