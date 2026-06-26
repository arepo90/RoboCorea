#include "gui/app_settings.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>

std::string AppSettings::settingsPath() const
{
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    // AppConfigLocation depends on the app/org name; pin a stable folder instead.
    QString base = QDir::homePath() + "/.config/robocorea_gui";
    (void)dir;
    QDir().mkpath(base);
    return (base + "/settings.json").toStdString();
}

std::vector<AppSettings::VideoStream> AppSettings::videoStreams()
{
    std::lock_guard<std::mutex> lk(video_mutex);
    return video_streams;
}

void AppSettings::load()
{
    QFile f(QString::fromStdString(settingsPath()));
    if (!f.open(QIODevice::ReadOnly))
        return;  // no settings yet → keep compiled-in defaults

    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    f.close();
    if (!doc.isObject())
        return;
    QJsonObject o = doc.object();

    if (o.contains("thermal_colormap"))  thermal_colormap.store(o["thermal_colormap"].toInt());
    if (o.contains("thermal_interp"))    thermal_interp.store(o["thermal_interp"].toInt());
    if (o.contains("thermal_upscale_w")) thermal_upscale_w.store(o["thermal_upscale_w"].toInt());
    if (o.contains("thermal_upscale_h")) thermal_upscale_h.store(o["thermal_upscale_h"].toInt());
    if (o.contains("label_font_scale_x100"))
        label_font_scale_x100.store(o["label_font_scale_x100"].toInt());
    if (o.contains("audio_start_enabled"))
        audio_start_enabled.store(o["audio_start_enabled"].toBool());
    {
        std::lock_guard<std::mutex> lk(strings_mutex);
        if (o.contains("vosk_grammar"))
            vosk_grammar = o["vosk_grammar"].toString().toStdString();
    }

    // PPM calibration: flat array of 18 (6 channels × {min, neutral, max}).
    if (o.contains("ppm_calib") && o["ppm_calib"].isArray()) {
        QJsonArray arr = o["ppm_calib"].toArray();
        if (arr.size() == 18) {
            std::lock_guard<std::mutex> lk(ppm_calib_mutex);
            for (int c = 0; c < 6; ++c) {
                ppm_calib[c].min_us     = arr[c * 3 + 0].toInt(1000);
                ppm_calib[c].neutral_us = arr[c * 3 + 1].toInt(1500);
                ppm_calib[c].max_us     = arr[c * 3 + 2].toInt(2000);
            }
        }
    }
    if (o.contains("ppm_deadband_1000"))
        ppm_deadband_1000 = o["ppm_deadband_1000"].toInt(50);

    std::lock_guard<std::mutex> lk(video_mutex);
    if (o.contains("default_robot_host"))
        default_robot_host = o["default_robot_host"].toString().toStdString();

    if (o.contains("video_streams") && o["video_streams"].isArray()) {
        std::vector<VideoStream> loaded;
        for (const auto& v : o["video_streams"].toArray()) {
            if (!v.isObject()) continue;
            QJsonObject s = v.toObject();
            VideoStream vs;
            vs.name       = s.value("name").toString().toStdString();
            vs.host       = s.value("host").toString().toStdString();
            vs.port       = s.value("port").toInt(8890);
            vs.latency_ms = s.value("latency_ms").toInt(120);
            vs.audio      = s.value("audio").toBool(false);
            if (!vs.name.empty())
                loaded.push_back(std::move(vs));
        }
        if (!loaded.empty())
            video_streams = std::move(loaded);
    }
}

void AppSettings::save()
{
    QJsonObject o;
    o["thermal_colormap"]      = thermal_colormap.load();
    o["thermal_interp"]        = thermal_interp.load();
    o["thermal_upscale_w"]     = thermal_upscale_w.load();
    o["thermal_upscale_h"]     = thermal_upscale_h.load();
    o["label_font_scale_x100"] = label_font_scale_x100.load();
    o["audio_start_enabled"]   = audio_start_enabled.load();
    {
        std::lock_guard<std::mutex> lk(strings_mutex);
        o["vosk_grammar"] = QString::fromStdString(vosk_grammar);
    }

    {
        std::lock_guard<std::mutex> lk(ppm_calib_mutex);
        QJsonArray arr;
        for (int c = 0; c < 6; ++c) {
            arr.append(ppm_calib[c].min_us);
            arr.append(ppm_calib[c].neutral_us);
            arr.append(ppm_calib[c].max_us);
        }
        o["ppm_calib"] = arr;
    }
    o["ppm_deadband_1000"] = ppm_deadband_1000.load();

    {
        std::lock_guard<std::mutex> lk(video_mutex);
        o["default_robot_host"] = QString::fromStdString(default_robot_host);
        QJsonArray arr;
        for (const auto& vs : video_streams) {
            QJsonObject s;
            s["name"]       = QString::fromStdString(vs.name);
            s["host"]       = QString::fromStdString(vs.host);
            s["port"]       = vs.port;
            s["latency_ms"] = vs.latency_ms;
            s["audio"]      = vs.audio;
            arr.append(s);
        }
        o["video_streams"] = arr;
    }

    QFile f(QString::fromStdString(settingsPath()));
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;
    f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
    f.close();
}
