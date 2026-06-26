#pragma once

#include <QByteArray>
#include <QDir>
#include <QString>

// Shared convention for the GUI-side map preview thumbnails. The map *data* lives
// on the robot (map_manager is authoritative); these PNGs are just the local
// preview the operator sees in the Maps panel — written by MapWindow on save,
// read by MapsPanel — mirroring how arm-pose thumbnails work.
//   kind is "2d" or "3d".
inline QString mapThumbDir()
{
    return QDir::homePath() + "/.config/robocorea_gui/map_thumbs";
}

inline QString mapThumbPath(const QString& name, const QString& kind)
{
    const QString key = QString::fromLatin1(
        name.toUtf8().toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
    return mapThumbDir() + "/" + key + "_" + kind + ".png";
}
