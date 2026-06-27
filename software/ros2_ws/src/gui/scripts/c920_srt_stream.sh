#!/bin/bash
# c920_srt_stream.sh — runs on the ROBOT (Jetson Orin Nano)
# =========================================================
# Streams the C920 webcams to the operator workstation over SRT, and (optionally)
# muxes a camera's MICROPHONE into the same stream as Opus audio.
#
# Why this shape:
#   • The Orin Nano has NO hardware video encoder (NVENC). We never encode video
#     on the Jetson — the C920 has an ONBOARD H.264 encoder (UVC H.264);
#     `uvch264src` pulls that already-compressed stream and we only packetize it.
#   • The C920 also has a microphone (a USB-audio capture device). We encode it
#     with OPUS (~24 kbps, packet-loss concealment built in) and mux video+audio
#     into ONE MPEG-TS so they share a single synchronized SRT connection.
#   • SRT carries it over the (degraded) Wi-Fi with selective retransmission
#     (ARQ) inside a fixed latency budget. The Jetson is the SRT LISTENER; the
#     GUI connects as caller and reconnects on drop.
#
# The RF analog driving cameras are NOT handled here — they are digitized at the
# workstation and appear there as plain /dev/videoN webcams.
#
# Usage:  ./c920_srt_stream.sh
# Edit the CAMERAS / tunables below to match your devices and link budget.

set -euo pipefail

# --- Configuration ------------------------------------------------------------
# "video_device ; srt_port ; alsa_audio_device"
#   Leave the audio field EMPTY for a video-only camera.
#   Find the C920's ALSA capture name with:  arecord -l   (look for "C920")
#   then use the stable form  hw:CARD=C920,DEV=0  (NOT hw:N — numbers move).
# NOTE: fields are ';'-separated because ALSA device strings contain ':'.
CAMERAS=(
    #"/dev/video0;8890;hw:CARD=C920,DEV=0"   # Front: video + mic (A/V)
    "/dev/video0;8890;"                      # Rear:  video only
)

WIDTH=1280
HEIGHT=720
FRAMERATE=30
BITRATE=2000          # video kbps per stream. Size to the link's WORST case.
KEYFRAME_MS=1000      # ms between IDR keyframes (SRT ARQ covers ordinary loss).
OPUS_KBPS=24          # audio bitrate. 24 kbps mono = clean voice.
LATENCY=120           # SRT receive/ARQ budget in ms.
PROFILE="constrained-baseline"   # most loss-tolerant + universally decodable

# --- Preflight ----------------------------------------------------------------
echo "Checking GStreamer + plugins…"
command -v gst-launch-1.0 >/dev/null || {
    echo "ERROR: gst-launch-1.0 not found."
    echo "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \\"
    echo "    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad"
    exit 1
}
for el in uvch264src srtsink mpegtsmux opusenc alsasrc; do
    gst-inspect-1.0 "$el" >/dev/null 2>&1 || {
        echo "ERROR: GStreamer element '$el' missing."
        echo "  sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-good"
        echo "  (plugins-bad provides srtsink/uvch264src/mpegtsmux/opusenc and pulls libsrt)"
        exit 1
    }
done

BR_BPS=$((BITRATE * 1000))
OPUS_BPS=$((OPUS_KBPS * 1000))

# --- Launch one pipeline per camera ------------------------------------------
PIDS=()
for entry in "${CAMERAS[@]}"; do
    IFS=';' read -r DEVICE PORT AUDIO_DEV <<< "$entry"
    if [ ! -e "$DEVICE" ]; then
        echo "WARNING: $DEVICE not found, skipping"
        continue
    fi

    VIDEO_CAPS="video/x-h264,width=${WIDTH},height=${HEIGHT},framerate=${FRAMERATE}/1,profile=${PROFILE}"
    SRT_URI="srt://:${PORT}?mode=listener&latency=${LATENCY}"

    if [ -n "$AUDIO_DEV" ]; then
        echo "Streaming $DEVICE + mic[$AUDIO_DEV] -> srt://0.0.0.0:${PORT}  (A/V)"
        gst-launch-1.0 -e \
            mpegtsmux name=mux alignment=7 ! \
            srtsink uri="$SRT_URI" wait-for-connection=false \
            uvch264src device="$DEVICE" \
                initial-bitrate=$BR_BPS average-bitrate=$BR_BPS peak-bitrate=$BR_BPS \
                rate-control=cbr iframe-period=$KEYFRAME_MS \
                name=src auto-start=true \
            src.vidsrc ! "$VIDEO_CAPS" ! h264parse config-interval=-1 ! queue ! mux. \
            alsasrc device="$AUDIO_DEV" ! audioconvert ! audioresample ! \
                audio/x-raw,rate=48000,channels=1 ! \
                opusenc bitrate=$OPUS_BPS audio-type=voice ! queue ! mux. \
            &
    else
        echo "Streaming $DEVICE -> srt://0.0.0.0:${PORT}  (video only)"
        gst-launch-1.0 -e \
            uvch264src device="$DEVICE" \
                initial-bitrate=$BR_BPS average-bitrate=$BR_BPS peak-bitrate=$BR_BPS \
                rate-control=cbr iframe-period=$KEYFRAME_MS \
                name=src auto-start=true \
            src.vidsrc ! "$VIDEO_CAPS" ! h264parse config-interval=-1 ! \
            mpegtsmux alignment=7 ! \
            srtsink uri="$SRT_URI" wait-for-connection=false \
            &
    fi
    PIDS+=($!)
done

[ ${#PIDS[@]} -gt 0 ] || { echo "No cameras started."; exit 1; }
echo ""
echo "Streams running (PIDs: ${PIDS[*]}). Ctrl+C to stop."

cleanup() {
    echo; echo "Shutting down…"
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup SIGINT SIGTERM
wait
