#pragma once

#include <QColor>
#include <QWidget>

#include <deque>

// Lightweight real-time strip chart for the magnetometer's three axes (X/Y/Z on
// ONE graph), so the operator can watch peaks/dips as the robot moves near
// ferrous / magnetic material.
//
// Deliberately a plain QWidget painted with QPainter (no signals/slots, no
// QtCharts dependency): the GUI already links only Qt6 Widgets/OpenGL, and a
// hand-rolled polyline keeps the build dependency-free. Keeps a bounded ring of
// the most recent samples per axis and auto-scales the Y axis to the visible
// window so both small wobble and large spikes stay readable. addSample() is
// called from onMagnetometerUpdated(); clear() is wired to the "Clear Data"
// button.
class MagPlot : public QWidget {
public:
    explicit MagPlot(QWidget* parent = nullptr);

    void addSample(double x, double y, double z);   // push one reading (µT)
    void clear();                                    // reset the graph

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    // ~600 points ≈ tens of seconds at typical mag rates; the window scrolls once
    // full. Bumping this only costs a little memory + paint time.
    static constexpr int kMaxPoints = 600;

    std::deque<double> x_, y_, z_;

    // Axis colors (also used for the legend). X warm-red, Y green, Z the same
    // cyan as the numeric readout.
    static const QColor kColorX;
    static const QColor kColorY;
    static const QColor kColorZ;
};
