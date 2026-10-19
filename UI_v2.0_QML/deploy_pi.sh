#!/usr/bin/env bash
# Adaptif Kalip UI (PySide6 + QML) - Pi'ye deploy + uzaktan dogrulama.
#
#   PI_HOST=berat@192.168.88.244 DEPLOY_BRANCH=ui_v2.0 ./deploy_pi.sh
#
# Ortam degiskenleri:
#   PI_HOST        (zorunlu) ssh hedefi (sabit gomulmez)
#   DEPLOY_BRANCH  (zorunlu) Pi'deki calisma kopyasinin BULUNMASI GEREKEN dal (kiosk: ui_v2.0).
#                  Pi farkli daldaysa script DURUR; dal degistirmez.
#   DEPLOY_COMMIT  (istege bagli) Pi'de olmasi beklenen tam/kisa commit. Verilmezse
#                  yereldeki refs/remotes/origin/<DEPLOY_BRANCH> kullanilir (yazdirilir).
#   PI_REPO        (istege bagli) Pi'deki repo yolu.
#
# Adimlar - herhangi biri basarisizsa script DURUR ve kiosk YENIDEN BASLATILMAZ:
#   1) Pi'deki dal == DEPLOY_BRANCH, izlenen dosyalarda yerel degisiklik yok
#   2) git fetch + ff-only birlestirme; Pi HEAD == beklenen commit
#   3) offscreen QML kontrolu (tests/qml_load_check.py, cikis kodu 0 olmali; kiosk'a
#      dokunmaz; Quick3D "not functional" uyarisi script icinde beklenen olarak suzulur)
#   4) kiosk yeniden baslatma
#   5) log (8 sn sonra); Traceback varsa hata koduyla cikar (kiosk calisiyor olabilir)
# Eski surumdeki "... | head -40 || true" hatayi yutuyordu: kontrol basarisiz olsa da
# restart yapiliyordu (inceleme 2026-10-19). Artik yok.
set -euo pipefail

fail() { echo "DURDU: $*" >&2; echo "Kiosk YENİDEN BAŞLATILMADI." >&2; exit 1; }

PI_HOST="${PI_HOST:-}"
BRANCH="${DEPLOY_BRANCH:-}"
EXPECTED="${DEPLOY_COMMIT:-}"
REPO="${PI_REPO:-/home/berat/Kinetic_Molding_System}"
SSH_OPTS=(-o ConnectTimeout=10 -o BatchMode=yes)

[ -n "$PI_HOST" ] || fail "PI_HOST ayarli degil. Ornek: PI_HOST=berat@192.168.88.244 DEPLOY_BRANCH=ui_v2.0 $0"
[ -n "$BRANCH" ] || fail "DEPLOY_BRANCH ayarli degil (kiosk dali, ornek: ui_v2.0)."
# uzak komut satirina girecek degerler: yalniz guvenli karakterler (enjeksiyon yok)
[[ "$BRANCH" =~ ^[A-Za-z0-9._/-]+$ ]] || fail "gecersiz dal adi: $BRANCH"
[[ "$REPO" =~ ^[A-Za-z0-9._/-]+$ ]] || fail "gecersiz PI_REPO: $REPO"

if [ -z "$EXPECTED" ]; then
    HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    EXPECTED="$(git -C "$HERE" rev-parse --verify -q "refs/remotes/origin/$BRANCH" || true)"
    [ -n "$EXPECTED" ] || fail "beklenen commit belirlenemedi (DEPLOY_COMMIT verin veya once git fetch)."
    echo "Beklenen commit (yerel origin/$BRANCH): $EXPECTED"
fi
[[ "$EXPECTED" =~ ^[0-9a-f]{7,40}$ ]] || fail "gecersiz DEPLOY_COMMIT: $EXPECTED"

remote() { ssh "${SSH_OPTS[@]}" "$PI_HOST" "$1"; }

echo "==> [1/5] Pi'de dal ve calisma kopyasi kontrolu ($PI_HOST)"
cur="$(remote "cd '$REPO' && git rev-parse --abbrev-ref HEAD")" || fail "Pi'ye baglanilamadi veya repo okunamadi."
[ "$cur" = "$BRANCH" ] || fail "Pi '$cur' dalinda; beklenen '$BRANCH'. Dal bu script tarafindan degistirilmez."
remote "cd '$REPO' && test -z \"\$(git status --porcelain --untracked-files=no)\"" \
    || fail "Pi'de izlenen dosyalarda yerel degisiklik var; once inceleyin."

echo "==> [2/5] Guncelle (fetch + ff-only) ve commit dogrula"
head="$(remote "cd '$REPO' && git fetch -q origin '$BRANCH' && git merge --ff-only -q FETCH_HEAD && git rev-parse HEAD")" \
    || fail "git fetch/ff-only birlestirme basarisiz."
case "$head" in
    "$EXPECTED"*) echo "Pi HEAD: $head" ;;
    *) fail "Pi HEAD ($head) beklenen commit ($EXPECTED) degil." ;;
esac

echo "==> [3/5] Offscreen QML kontrolu (kiosk'a dokunmadan)"
remote "cd '$REPO/UI_v2.0_QML' && QT_QPA_PLATFORM=offscreen timeout 90 venv/bin/python3 tests/qml_load_check.py" \
    || fail "QML kontrolu basarisiz (yukaridaki UYARI satirlari)."

echo "==> [4/5] Kiosk yeniden baslatiliyor"
remote "sudo systemctl restart getty@tty1" || fail "kiosk yeniden baslatilamadi."

echo "==> [5/5] Log (8 sn bekle)"
sleep 8
log="$(remote "tail -n 60 /home/berat/kalip.log")" || { echo "UYARI: log okunamadi." >&2; exit 2; }
printf '%s\n' "$log"
if printf '%s\n' "$log" | grep -q "Traceback"; then
    echo "HATA: kiosk logunda Traceback var (kiosk yeniden baslatildi; inceleyin)." >&2
    exit 3
fi
echo "==> Deploy tamam: $BRANCH @ $head"
