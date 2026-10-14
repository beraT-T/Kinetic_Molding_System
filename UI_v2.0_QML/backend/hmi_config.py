"""
Merkezi sabitler (tek yer) + config/machine.json yukleyici.

Degerlerin gerekcesi ve onay durumu: docs/v2.2/KARARLAR.md (K1..).
Operatore gorunen metinler Turkce karakterli; kod yorumlari ASCII.
"""
import json
import os

from stl_interpolator import SAFE_MIN_MM, SAFE_MAX_MM, STROKE_MAX_MM  # noqa: F401

# ---------------- hedef / tolerans ----------------
TARGET_TOL_MM = 2          # K1: ARR/ALL/MOV hedef toleransi (STAT mm tamsayi)
HOME_ZERO_TOL_MM = 2       # K2: HOME sonrasi "sifirda" sayilma toleransi
MOTORS_PER_MODULE = 9
MAX_MODULES = 16

# ---------------- seri hat / zamanlama ----------------
# RS485 9600 baud ~960 bayt/sn. STAT sorgu+cevap ~63 bayt ~66 ms; U3 cevap vermeden
# once U2'yi en fazla 1500 ms bekler (firmware U2_Cmd timeout) -> cevap <= ~1,6 sn.
POLL_TICK_MS = 1000              # gozetim poll'u: tick basina TEK modul (round-robin)
STAT_TIMEOUT_MS = 2500           # STAT cevabi icin bekleme (1,5 sn U2 + hat + pay)
MOTION_ACK_TIMEOUT_MS = 2500     # ARR/ALL/MOV/HOME kabul (ACK/ERR) icin bekleme
PING_TIMEOUT_MS = 300            # PING yerel cevaplanir (U2'ye gitmez; ~30-70 ms)
PING_GUARD_MS = 100              # PING zaman asimi sonrasi kisa koruma (tarama suresi)
QUERY_TIMEOUT_MS = 2500          # GETPOS/GETID/SETID
INTER_FRAME_MS = 20              # cevap sonrasi hatta yeni yazmadan once bosluk
POST_TIMEOUT_GUARD_MS = 300      # zaman asimindan sonra gec cevap icin hatti dinle
QUEUE_MAX = 64                   # seri istek kuyrugu ust siniri
COMM_FAIL_LIMIT = 2              # ardisik cevapsiz istek -> modul erisilemez
SYNC_CONFIRM = 2                 # K11: senkron disi modul icin ardisik tutarli taze STAT
OP_PHASE_TIMEOUT_S = 300.0       # faz basina (home ~200 sn, hareket ~150 sn + pay)

# ---------------- seri port (SerialManager) ----------------
SERIAL_WRITE_TIMEOUT_S = 0.5     # yazma takilirsa UI degil yazici thread bekler
SERIAL_MAX_LINE = 512            # satir sonu gelmeyen tampon siniri (bayt)
SERIAL_LINE_QUEUE_MAX = 256      # okuyucu -> ana thread satir kuyrugu siniri

# ---------------- veri yasi (K10) ----------------
STALE_MIN_S = 10.0


def worst_poll_round_s(n_modules):
    """Bir modulun iki STAT'i arasindaki en kotu sure (her istek zaman asimina
    ugrasa bile): n * max(tick, STAT zaman asimi + koruma)."""
    n = max(1, int(n_modules))
    per = max(POLL_TICK_MS, STAT_TIMEOUT_MS + POST_TIMEOUT_GUARD_MS) / 1000.0
    return n * per


def data_stale_s(n_modules):
    """K10: veri yasi esigi = en kotu tur suresi x 2 (alt sinir STALE_MIN_S)."""
    return max(STALE_MIN_S, 2.0 * worst_poll_round_s(n_modules))


def worst_alarm_latency_s(n_modules):
    """Bir motor F'ye dustukten sonra operatorun gormesine kadar en kotu sure:
    modulun bir sonraki sorgusu (en kotu tur) + o sorgunun cevabi."""
    return worst_poll_round_s(n_modules) + STAT_TIMEOUT_MS / 1000.0


def typical_alarm_latency_s(n_modules):
    """Butun moduller cevap verirken: n * tick + tipik cevap (~0,1 sn)."""
    return max(1, int(n_modules)) * POLL_TICK_MS / 1000.0 + 0.1


# ---------------- makine yapilandirmasi ----------------
FIRMWARE_VARIANTS = ("v5.1", "v5f411")
# U2 kartindaki motorlar (host numarasi). U2 cevap vermezse bu motorlar bilinmez.
U2_MOTORS = {"v5.1": (1, 2, 3, 4, 5), "v5f411": (5, 6, 7, 8, 9)}
U3_MOTORS = {"v5.1": (6, 7, 8, 9), "v5f411": (1, 2, 3, 4)}

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "machine.json")


def load_machine_config(path=None):
    """machine.json oku. Donus: {expected_modules: list|None,
    firmware_variant: str|None, errors: [str], path: str}.
    Hatali alan -> None (yapilandirilmamis) + hata metni (muhafazakar)."""
    path = path or DEFAULT_CONFIG_PATH
    out = {"expected_modules": None, "firmware_variant": None, "errors": [], "path": path}
    if not os.path.exists(path):
        out["errors"].append("machine.json bulunamadı; varsayılan (yapılandırılmamış) kullanılıyor.")
        return out
    try:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    except Exception as e:  # bozuk dosya: hicbir sey varsayma
        out["errors"].append(f"machine.json okunamadı: {e}")
        return out
    exp = raw.get("expected_modules")
    if exp is not None:
        ok = (isinstance(exp, list) and exp and
              all(isinstance(v, int) and 1 <= v <= MAX_MODULES for v in exp) and
              len(set(exp)) == len(exp))
        if ok:
            out["expected_modules"] = sorted(exp)
        else:
            out["errors"].append("expected_modules geçersiz (1–16 arası benzersiz tam sayı listesi olmalı).")
    var = raw.get("firmware_variant")
    if var is not None:
        if var in FIRMWARE_VARIANTS:
            out["firmware_variant"] = var
        else:
            out["errors"].append("firmware_variant geçersiz (\"v5.1\" veya \"v5f411\" olmalı).")
    return out
