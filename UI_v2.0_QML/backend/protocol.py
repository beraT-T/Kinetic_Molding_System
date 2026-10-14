"""
Protokol v5.1 - saf (Qt'siz) yardimcilar: cevap ayristirma, KATI STAT semasi,
terminal fiil ayristirma, hedef dogrulama. Hepsi birim test edilebilir.

STAT token sirasi firmware kaynagindan dogrulandi (yalniz okundu):
  v5f411 U3: "STAT:id:<M1..M4>,<M5..M9>"  ; U2 cevapsizsa "STAT:id:<M1..M4>,U2ERR"
  v5.1   U3: "STAT:id:<M1..M5>,<M6..M9>"  ; U2 cevapsizsa "STAT:id:U2ERR,<M6..M9>"
Token: <harf><mm>, harf I/M/H/S/F. Katı sema: ^[IMHSF]\\d+$ (eksi deger = gecersiz).
"""
import re

from hmi_config import (SAFE_MIN_MM, SAFE_MAX_MM, MOTORS_PER_MODULE, MAX_MODULES,
                        U2_MOTORS, U3_MOTORS)

TOKEN_RE = re.compile(r"^([IMHSF])(\d+)$")

ACK_VERBS = {
    "ARR": ("ARROK", "ARRERR"),
    "ALL": ("ALLOK", "ALLERR"),
    "MOV": ("MOVOK", "MOVERR"),
    "HOME": ("HOMEOK", "HOMEERR"),
}
MOTION_VERBS = ("ARR", "ALL", "MOV", "HOME")
FREE_VERBS = ("PING", "STAT", "GETPOS", "GETID")
LOCKED_VERBS = ("SETID",)


# ======================= cevap ayristirma =======================
class Reply:
    __slots__ = ("raw", "verb", "sid", "fields")

    def __init__(self, raw, verb, sid, fields):
        self.raw, self.verb, self.sid, self.fields = raw, verb, sid, fields

    def __repr__(self):
        return f"Reply({self.raw!r})"


def _int_or_none(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return None


def parse_reply(line):
    """Gelen satiri ayristir. Ilk alan fiil, ikinci alan (varsa, sayisal ise) modul id.
    STAT govdesi tek alan olarak tutulur (virgullu)."""
    line = (line or "").strip()
    if not line:
        return None
    if line.startswith("STAT:"):
        parts = line.split(":", 2)
        sid = _int_or_none(parts[1]) if len(parts) > 1 else None
        body = parts[2] if len(parts) > 2 else ""
        return Reply(line, "STAT", sid, [body])
    parts = line.split(":")
    verb = parts[0]
    sid = _int_or_none(parts[1]) if len(parts) > 1 else None
    return Reply(line, verb, sid, parts[2:])


def reply_motor(reply):
    """MOVOK/HOMEOK/POS... cevabindaki motor no (yoksa None)."""
    if reply.fields:
        return _int_or_none(reply.fields[0])
    return None


# ======================= KATI STAT =======================
def parse_token(tok):
    """'S300' -> ('S', 300); gecersiz -> None."""
    m = TOKEN_RE.match((tok or "").strip())
    if not m:
        return None
    return m.group(1), int(m.group(2))


def interpret_stat(body, variant=None):
    """STAT govdesini 9 motora esle.

    Donus sozlugu:
      motors          : 9 eleman, her biri (harf, mm) veya None (bilinmiyor)
      complete        : 9 gecerli token (tam ve gecerli cevap)
      u2err           : U2 iletisim hatasi bildirildi
      unmapped_faults : motor numarasi belirlenemeyen F sayisi
      variant_mismatch: U2ERR konumu yapilandirilan surumle celisiyor
      malformed       : sema disi / eksik token (U2ERR haric)
    """
    toks = [t.strip() for t in (body or "").split(",")] if body else []
    out = {"motors": [None] * MOTORS_PER_MODULE, "complete": False, "u2err": False,
           "unmapped_faults": 0, "variant_mismatch": False, "malformed": False}

    def count_faults(ts):
        return sum(1 for t in ts if (parse_token(t) or ("", 0))[0] == "F")

    if len(toks) == MOTORS_PER_MODULE and "U2ERR" not in toks:
        parsed = [parse_token(t) for t in toks]
        out["motors"] = parsed
        out["complete"] = all(p is not None for p in parsed)
        out["malformed"] = not out["complete"]
        return out

    if "U2ERR" in toks and len(toks) == 5:
        out["u2err"] = True
        first = toks[0] == "U2ERR"
        last = toks[-1] == "U2ERR"
        local = toks[1:] if first else toks[:-1]
        fmt = "v5.1" if first else ("v5f411" if last else None)
        if variant is None or fmt is None or fmt != variant:
            # K4: surum bilinmiyor (veya celiskili) -> yerel tokenlarin hangi motora
            # ait oldugu BELIRLENEMEZ. F'ler kaybolmaz: numarasiz ariza olarak sayilir.
            out["variant_mismatch"] = variant is not None and fmt != variant
            out["unmapped_faults"] = count_faults(local)
            return out
        for motor, tok in zip(U3_MOTORS[variant], local):
            out["motors"][motor - 1] = parse_token(tok)
        return out

    # beklenmeyen sayi/bicim: hicbir motor bilinmiyor; gecerli F'ler kaybolmaz
    out["malformed"] = True
    out["u2err"] = "U2ERR" in toks
    out["unmapped_faults"] = count_faults(toks)
    return out


def u2_motor_text(variant):
    if variant in U2_MOTORS:
        ms = U2_MOTORS[variant]
        return f"motor {ms[0]}–{ms[-1]}"
    return "U2'ye bağlı motorlar"


# ======================= hedef dogrulama =======================
def check_target(text):
    """Operator hedef girdisi. "" = gecerli; aksi halde operatore gosterilecek sebep."""
    s = str(text if text is not None else "").strip()
    if not s:
        return "Hedef boş. Bir değer girin."
    if not re.fullmatch(r"-?\d+", s):
        return "Hedef tam sayı (mm) olmalı."
    v = int(s)
    if v < SAFE_MIN_MM or v > SAFE_MAX_MM:
        return (f"Hedef {SAFE_MIN_MM}–{SAFE_MAX_MM} mm aralığında olmalı "
                f"(tam strok sınırına komut verilmez).")
    return ""


def check_target_value(v):
    try:
        iv = int(v)
    except (TypeError, ValueError):
        return "Hedef tam sayı (mm) olmalı."
    if isinstance(v, float) and v != iv:
        return "Hedef tam sayı (mm) olmalı."
    return check_target(str(iv))


# ======================= terminal =======================
class TerminalCmd:
    __slots__ = ("text", "verb", "sid", "motor", "values", "error")

    def __init__(self, text, verb=None, sid=None, motor=None, values=None, error=""):
        self.text, self.verb, self.sid = text, verb, sid
        self.motor, self.values, self.error = motor, values or [], error


def _sid_ok(s):
    v = _int_or_none(s)
    return v if v is not None and 1 <= v <= MAX_MODULES and s.isdigit() else None


def _motor_ok(s):
    v = _int_or_none(s)
    return v if v is not None and 1 <= v <= MOTORS_PER_MODULE and s.isdigit() else None


def parse_terminal(text):
    """Terminal satirini ayristir ve yapisini dogrula. error != "" ise gonderilmez."""
    t = (text or "").strip().upper()
    c = TerminalCmd(t)
    if not t:
        c.error = "Boş komut."
        return c
    parts = t.split(":")
    c.verb = parts[0]
    allowed = FREE_VERBS + MOTION_VERBS
    if c.verb in LOCKED_VERBS:
        c.error = ("SETID kilitli: modül kimliğini flash'a yazar. "
                   "Bu arayüzden gönderilmez.")
        return c
    if c.verb not in allowed:
        c.error = (f"Tanınmayan komut: {c.verb}. İzinli: " + ", ".join(allowed) + ".")
        return c
    if len(parts) < 2 or _sid_ok(parts[1]) is None:
        c.error = "Modül numarası 01–16 olmalı (ör. STAT:01)."
        return c
    c.sid = _sid_ok(parts[1])
    rest = parts[2:]
    v = c.verb
    if v in ("PING", "STAT", "GETID"):
        if rest:
            c.error = f"Biçim: {v}:<modül>"
        return c
    if v == "GETPOS":
        if len(rest) != 1 or _motor_ok(rest[0]) is None:
            c.error = "Biçim: GETPOS:<modül>:<motor 1–9>"
        else:
            c.motor = _motor_ok(rest[0])
        return c
    if v == "HOME":
        if not rest:
            return c
        if len(rest) == 1 and rest[0] in ("0", "00"):
            return c
        if len(rest) != 1 or _motor_ok(rest[0]) is None:
            c.error = "Biçim: HOME:<modül> veya HOME:<modül>:<motor 1–9>"
        else:
            c.motor = _motor_ok(rest[0])
        return c
    if v == "MOV":
        if len(rest) != 2 or _motor_ok(rest[0]) is None:
            c.error = "Biçim: MOV:<modül>:<motor 1–9>:<mm>"
            return c
        c.motor = _motor_ok(rest[0])
        err = check_target(rest[1])
        if err:
            c.error = err
        else:
            c.values = [int(rest[1])]
        return c
    if v == "ALL":
        if len(rest) != 1:
            c.error = "Biçim: ALL:<modül>:<mm>"
            return c
        err = check_target(rest[0])
        if err:
            c.error = err
        else:
            c.values = [int(rest[0])]
        return c
    if v == "ARR":
        if len(rest) != MOTORS_PER_MODULE:
            c.error = "Biçim: ARR:<modül>:<9 hedef mm>"
            return c
        for r in rest:
            err = check_target(r)
            if err:
                c.error = err
                return c
        c.values = [int(r) for r in rest]
        return c
    return c


# ======================= komut metinleri =======================
def cmd_arr(sid, vals):
    return f"ARR:{sid:02d}:" + ":".join(str(int(v)) for v in vals)


def cmd_all(sid, mm):
    return f"ALL:{sid:02d}:{int(mm)}"


def cmd_mov(sid, motor, mm):
    return f"MOV:{sid:02d}:{motor:02d}:{int(mm)}"


def cmd_home(sid, motor=0):
    return f"HOME:{sid:02d}" if not motor else f"HOME:{sid:02d}:{motor:02d}"


def cmd_stat(sid):
    return f"STAT:{sid:02d}"


def cmd_ping(sid):
    return f"PING:{sid:02d}"


def request_matches(verb, sid, motor, reply):
    """Bekleyen istek (verb, sid, motor) bu cevaba mi ait?
    Protokolde islem kimligi YOK: eslesme fiil + modul (+ motor) ile yapilir.
    Kalan risk (gec gelen eski cevap): docs/v2.2/PROTOKOL_ONERI_v5.2.md."""
    if reply is None:
        return False
    rv = reply.verb
    if verb == "STAT":
        return rv == "STAT" and reply.sid == sid
    if verb == "PING":
        return rv == "PONG" and reply.sid == sid
    if verb == "GETID":
        return rv == "IDVAL" and reply.sid == sid
    if verb == "SETID":
        return rv in ("IDSET", "IDERR")
    if verb == "GETPOS":
        return rv in ("POS", "POSERR") and reply.sid == sid and reply_motor(reply) == motor
    if verb == "ARR":
        if rv == "ARRERR" and reply.sid is None:      # "ARRERR:FMT" (id'siz)
            return True
        return rv in ACK_VERBS["ARR"] and reply.sid == sid
    if verb == "ALL":
        return rv in ACK_VERBS["ALL"] and reply.sid == sid
    if verb == "MOV":
        return rv in ACK_VERBS["MOV"] and reply.sid == sid and reply_motor(reply) == motor
    if verb == "HOME":
        return (rv in ACK_VERBS["HOME"] and reply.sid == sid and
                (reply_motor(reply) or 0) == (motor or 0))
    return False


def is_ack_ok(reply):
    return reply is not None and reply.verb.endswith("OK")
