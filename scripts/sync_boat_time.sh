#!/usr/bin/env bash
# ==============================================================================
# sync_boat_time.sh
# Sends laptop UTC time and TimeZone automatically to USV boat via SSH
# Usage: ./sync_boat_time.sh
# ==============================================================================

TZ_NAME="$(cat /etc/timezone 2>/dev/null || timedatectl show -p Timezone --value 2>/dev/null || echo 'Europe/Madrid')"
CURRENT_UTC="$(date -u +'%Y-%m-%d %H:%M:%S')"

echo "=========================================================="
echo " Synchronizing USV Clock from GCS Laptop"
echo " Laptop UTC: ${CURRENT_UTC}"
echo " TimeZone:   ${TZ_NAME}"
echo "=========================================================="

ssh usv5 "sudo timedatectl set-timezone ${TZ_NAME} 2>/dev/null; sudo date -u -s '${CURRENT_UTC}'"

echo "[OK] USV Clock synchronized successfully!"
