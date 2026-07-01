#!/bin/bash
# jetson_speaker_sink.sh — runs on the ROBOT (Jetson Orin Nano)
# =========================================================
# Robot end of the operator -> robot TALKBACK path (the reverse of the C920 A/V
# audio). Receives the operator workstation's microphone over SRT (Opus in
# MPEG-TS, exactly the reverse of c920_srt_stream.sh) and plays it on a
# Jetson-attached speaker.
#
# Direction of the SRT handshake mirrors the video path: the ROBOT is the SRT
# *listener*, the GUI (GstMicSender) is the *caller*. Data flows caller -> here.
#
# ── Speaker selection (IMPLEMENTATION PENDING) ────────────────────────────────
# The output device is intentionally a single variable so the hardware choice can
# be made later without touching anything else:
#
#   • SINK="autoaudiosink"                       ← default: system default output.
#       Works out of the box for a USB speaker or a Bluetooth speaker that is
#       paired AND selected as the default sink (PipeWire/PulseAudio).
#   • USB speaker, pin explicitly:
#       SINK="alsasink device=hw:CARD=UACDemo"   (find the CARD with `aplay -l`)
#   • Bluetooth speaker, pin explicitly (PipeWire/Pulse):
#       SINK="pulsesink device=bluez_output.AA_BB_CC_DD_EE_FF.1"
#       (find it with `pactl list short sinks`)
#
# Bluetooth auto-reconnect: pairing is one-time, but auto-connect on boot is a
# BlueZ/session concern, NOT this script. Trust the device once
# (`bluetoothctl` -> `trust AA:BB:...`) so it reconnects when in range. Because
# Bluetooth/PipeWire live in the user session, prefer running this as a
# `systemd --user` service (see systemd/jetson-speaker.service + README) so it
# has access to the session's audio sink. A USB speaker has no such requirement.

set -euo pipefail

# --- Configuration ------------------------------------------------------------
PORT=8892                 # must match AppSettings::talkback_port in the GUI
LATENCY=120               # SRT receive/ARQ budget in ms (match the sender)
SINK="autoaudiosink"      # override per the notes above once the speaker is set

# --- Preflight ----------------------------------------------------------------
command -v gst-launch-1.0 >/dev/null || {
    echo "ERROR: gst-launch-1.0 not found."
    echo "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \\"
    echo "    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad"
    exit 1
}
for el in srtsrc tsdemux opusparse opusdec; do
    gst-inspect-1.0 "$el" >/dev/null 2>&1 || {
        echo "ERROR: GStreamer element '$el' missing (opus = plugins-base, srt/ts = plugins-bad)."
        exit 1
    }
done

SRT_URI="srt://:${PORT}?mode=listener&latency=${LATENCY}"
echo "Talkback speaker: listening on ${SRT_URI}  ->  ${SINK}"

# ${SINK} is intentionally unquoted so "alsasink device=..." splits into tokens.
exec gst-launch-1.0 -e \
    srtsrc uri="$SRT_URI" ! \
    tsdemux ! opusparse ! opusdec plc=true ! \
    audioconvert ! audioresample ! \
    ${SINK}
