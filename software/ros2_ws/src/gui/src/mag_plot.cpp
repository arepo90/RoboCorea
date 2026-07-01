#include "gui/mag_plot.hpp"

#include <QPainter>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

const QColor MagPlot::kColorX{"#ff6b6b"};   // X — red
const QColor MagPlot::kColorY{"#5cd65c"};   // Y — green
const QColor MagPlot::kColorZ{"#4fc3f7"};   // Z — cyan (matches numeric readout)

MagPlot::MagPlot(QWidget* parent) : QWidget(parent)
{
    setMinimumHeight(150);
    setToolTip("Live magnetometer — X/Y/Z in µT. Y auto-scales to the window; the "
               "trace scrolls left once full.");
}

void MagPlot::addSample(double x, double y, double z)
{
    auto push = [](std::deque<double>& d, double v) {
        d.push_back(v);
        while (static_cast<int>(d.size()) > kMaxPoints)
            d.pop_front();
    };
    push(x_, x);
    push(y_, y);
    push(z_, z);
    update();
}

void MagPlot::clear()
{
    x_.clear();
    y_.clear();
    z_.clear();
    update();
}

void MagPlot::paintEvent(QPaintEvent* /*event*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor("#1a1a2e"));

    const qreal ml = 6, mr = 6, mt = 6, mb = 6;   // margins
    QRectF plot(ml, mt, width() - ml - mr, height() - mt - mb);
    p.setPen(QColor("#333"));
    p.drawRect(plot);

    if (x_.empty() && y_.empty() && z_.empty()) {
        p.setPen(QColor("#666"));
        p.drawText(plot, Qt::AlignCenter, "Waiting for data…");
        return;
    }

    // Y range across all three axes in the window, with a 10% margin so the
    // extremes don't touch the frame.
    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const auto* d : {&x_, &y_, &z_})
        for (double v : *d) {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    if (hi - lo < 1e-6) {   // flat / single sample → give it some room
        lo -= 1.0;
        hi += 1.0;
    }
    const double margin = (hi - lo) * 0.10;
    lo -= margin;
    hi += margin;
    const double span = hi - lo;

    auto yPix = [&](double v) {
        return plot.bottom() - (v - lo) / span * plot.height();
    };

    // Zero reference line (only when the window straddles zero).
    if (lo < 0.0 && hi > 0.0) {
        p.setPen(QPen(QColor("#444"), 1, Qt::DashLine));
        const qreal y0 = yPix(0.0);
        p.drawLine(QPointF(plot.left(), y0), QPointF(plot.right(), y0));
    }

    // Range labels (top = hi, bottom = lo).
    p.setPen(QColor("#666"));
    QFont small = p.font();
    small.setPointSize(7);
    p.setFont(small);
    p.drawText(QPointF(plot.left() + 2, plot.top() + 10),
               QString::number(hi, 'f', 1));
    p.drawText(QPointF(plot.left() + 2, plot.bottom() - 3),
               QString::number(lo, 'f', 1));

    // Traces — samples stretched to fill the width (newest on the right).
    struct Axis {
        const std::deque<double>* d;
        QColor c;
        const char* name;
    };
    const std::array<Axis, 3> axes{{
        {&x_, kColorX, "X"},
        {&y_, kColorY, "Y"},
        {&z_, kColorZ, "Z"},
    }};

    for (const auto& a : axes) {
        const auto& d = *a.d;
        if (d.size() < 2)
            continue;
        const int n = static_cast<int>(d.size());
        QPolygonF poly;
        poly.reserve(n);
        for (int i = 0; i < n; ++i) {
            const qreal xp = plot.left() +
                             static_cast<double>(i) / (n - 1) * plot.width();
            poly << QPointF(xp, yPix(d[i]));
        }
        p.setPen(QPen(a.c, 1.5));
        p.drawPolyline(poly);
    }

    // Legend + latest value per axis, top-right.
    qreal lx = plot.right() - 92;
    qreal ly = plot.top() + 10;
    for (const auto& a : axes) {
        p.setPen(QPen(a.c, 2));
        p.drawLine(QPointF(lx, ly - 3), QPointF(lx + 12, ly - 3));
        p.setPen(a.c);
        const QString v = a.d->empty() ? QString("--")
                                       : QString::number(a.d->back(), 'f', 1);
        p.drawText(QPointF(lx + 16, ly), QString("%1 %2").arg(a.name, v));
        ly += 12;
    }
}
