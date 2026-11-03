#!/usr/bin/env bash
# ==============================================================================
# sync_boat_time.sh
# Synchronizes USV boat system clock with GCS laptop using identical local time.
# ==============================================================================

LAPTOP_LOCAL="$(date +'%Y-%m-%d %H:%M:%S')"

echo "=========================================================="
echo " Synchronizing USV Clock from GCS Laptop"
echo " Laptop Local Time: ${LAPTOP_LOCAL}"
echo "=========================================================="

ssh -t usv5 "sudo date -s '${LAPTOP_LOCAL}'"

echo "[OK] USV Clock synchronized to exact laptop local time!"
