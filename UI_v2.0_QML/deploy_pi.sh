#!/usr/bin/env bash
# Adaptif Kalip UI v2.0 - Pi'ye deploy + uzaktan dogrulama.
# Host sabit gomulmez; PI_HOST env'den okunur.
#   PI_HOST=berat@192.168.88.244 ./deploy_pi.sh
# Adimlar: git pull --ff-only -> offscreen QML kontrolu -> kiosk restart -> log.
set -euo pipefail

PI_HOST="${PI_HOST:-}"
if [ -z "$PI_HOST" ]; then
    echo "HATA: PI_HOST ayarli degil. Ornek: PI_HOST=berat@192.168.88.244 $0" >&2
    exit 1
fi
REPO="${PI_REPO:-/home/berat/Kinetic_Molding_System}"
SSH_OPTS="-o ConnectTimeout=10 -o BatchMode=yes"

echo "==> [1/4] Pi'de guncelle ($PI_HOST)"
ssh $SSH_OPTS "$PI_HOST" "cd '$REPO' && git fetch origin ui_v2.0 -q && git pull --ff-only -q && git log --oneline -1"

echo "==> [2/4] Offscreen QML kontrolu (kiosk'a dokunmadan)"
# Quick3D cizilmez; 'isApiRhiBased ... not functional' BEKLENEN ve zararsiz.
ssh $SSH_OPTS "$PI_HOST" "cd '$REPO/UI_v2.0_QML' && \
  QT_QPA_PLATFORM=offscreen timeout 20 venv/bin/python3 main.py 2>&1 | \
  grep -viE 'isApiRhiBased|not functional|not going to display|Populating font' | head -40 || true"

echo "==> [3/4] Kiosk yeniden baslatiliyor"
ssh $SSH_OPTS "$PI_HOST" "sudo systemctl restart getty@tty1"

echo "==> [4/4] Log (8sn bekle)"
sleep 8
ssh $SSH_OPTS "$PI_HOST" "tail -n 60 /home/berat/kalip.log"

echo "==> Deploy tamam."
