#!/bin/bash
# srt_stream.sh — runs on the ROBOT (Jetson Orin Nano)
# =========================================================
# Streams a standard webcam to the operator workstation over SRT, and (optionally)
# muxes a microphone into the same stream as Opus audio.
#
# Hardware notes: 
#   • The target camera lacks hardware H.264 encoding.
#   • We pull MJPEG from the camera to save USB bandwidth.
#   • The Jetson Orin Nano CPU (x264enc on ultrafast/zerolatency) is used 
#     to encode the video to H.264 on the fly.

set -euo pipefail

# --- Configuration ------------------------------------------------------------
CAMERAS=(
    #"/dev/video2;8890;hw:CARD=C920,DEV=0"   # Example: video + mic (A/V)
    "/dev/video2;8890;hw:CARD=C920,DEV=0"                      # Current: video only
)

WIDTH=1280
HEIGHT=720
FRAMERATE=20
BITRATE=1500          # Video kbps per stream. Size to the link's WORST case.
OPUS_KBPS=24          # Audio bitrate. 24 kbps mono = clean voice.
LATENCY=120           # SRT receive/ARQ budget in ms.
PROFILE="constrained-baseline"

# --- Preflight ----------------------------------------------------------------
echo "Checking GStreamer + plugins…"
command -v gst-launch-1.0 >/dev/null || {
    echo "ERROR: gst-launch-1.0 not found."
    echo "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \\"
    echo "    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly"
    exit 1
}
for el in v4l2src jpegdec x264enc srtsink mpegtsmux opusenc alsasrc; do
    gst-inspect-1.0 "$el" >/dev/null 2>&1 || {
        echo "ERROR: GStreamer element '$el' missing."
        echo "  Ensure you have x264enc (plugins-ugly), jpegdec (plugins-good), and srtsink (plugins-bad) installed."
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

    SRT_URI="srt://:${PORT}?mode=listener&latency=${LATENCY}"

    if [ -n "$AUDIO_DEV" ]; then
        echo "Streaming $DEVICE + mic[$AUDIO_DEV] -> srt://0.0.0.0:${PORT}  (A/V, CPU encoded)"
        gst-launch-1.0 -e \
            mpegtsmux name=mux alignment=7 ! \
            srtsink uri="$SRT_URI" wait-for-connection=false \
            v4l2src device="$DEVICE" ! \
            image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FRAMERATE}/1 ! \
            jpegdec ! videoconvert ! \
            x264enc bitrate=$BITRATE speed-preset=ultrafast tune=zerolatency key-int-max=$FRAMERATE ! \
            video/x-h264,profile=$PROFILE ! h264parse config-interval=-1 ! queue ! mux. \
            alsasrc device="$AUDIO_DEV" ! audioconvert ! audioresample ! \
                audio/x-raw,rate=48000,channels=1 ! \
                opusenc bitrate=$OPUS_BPS audio-type=voice ! queue ! mux. \
            &
    else
        echo "Streaming $DEVICE -> srt://0.0.0.0:${PORT}  (video only, CPU encoded)"
        gst-launch-1.0 -e \
            v4l2src device="$DEVICE" ! \
            image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FRAMERATE}/1 ! \
            jpegdec ! videoconvert ! \
            x264enc bitrate=$BITRATE speed-preset=ultrafast tune=zerolatency key-int-max=$FRAMERATE ! \
            video/x-h264,profile=$PROFILE ! h264parse config-interval=-1 ! \
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