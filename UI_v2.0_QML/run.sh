#!/usr/bin/env bash
# Adaptif Kalip - QML UI baslatici.
# Calistirma: ./run.sh            (pencere)
#             ./run.sh --fullscreen (tam ekran / panel)
# Nereden cagrilirsa cagrilsin dogru klasor + dogru python kullanilir.
set -e

# Bu scriptin bulundugu klasor (cwd'den bagimsiz)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# venv varsa onun python'unu kullan (Mac); yoksa sistem python3 (Pi - deps sistem geneli)
if [ -x "$DIR/venv/bin/python" ]; then
    PY="$DIR/venv/bin/python"
else
    PY="python3"
fi

exec "$PY" "$DIR/main.py" "$@"
