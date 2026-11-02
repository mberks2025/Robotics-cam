#!/usr/bin/env bash
# Log Jetson RAM / CPU / GPU once a second to a file, and print the peak RAM on Ctrl+C.
# tegrastats counts the shared RAM the GPU uses; psutil alone under-counts it.
#   ./scripts/log_tegrastats.sh yolo_fp16_640.log
OUT="${1:-tegrastats_$(date +%Y%m%d_%H%M%S).log}"
echo "Logging to $OUT (Ctrl+C to stop)"
trap 'echo; echo "Peak RAM: $(grep -oE "RAM [0-9]+/[0-9]+MB" "$OUT" | sort -t" " -k2 -n | tail -1)"; exit 0' INT
sudo tegrastats --interval 1000 | tee "$OUT"
