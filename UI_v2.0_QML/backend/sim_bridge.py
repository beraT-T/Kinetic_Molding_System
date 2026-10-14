"""
Donanimsiz simulasyon koprusu (U3 slave modullerinin protokol v5.1 davranisi).

Iki kullanimi var:
  1) --demo modu: gercek protokol yolu (kuyruk, ACK, STAT, alarm) donanimsiz calisir.
  2) Testler: hata enjeksiyonu (yazma hatasi, kayip ACK, gecikme, bozuk satir,
     kismi STAT/U2ERR, eski/sira disi cevap, slave reset, kopma).

Firmware referansi (yalniz okundu): Adaptif_kalip_v5f411_U3_4motor/src/main.cpp,
Adaptif_kalip_v5_U3_4motor/src/main.cpp. Simulasyon BIREBIR firmware degildir;
zamanlama ve durum harfleri yaklasik modellenir. Gercek hat/RS485 zamanlamasi,
motor fizigi ve guc davranisi BURADA DOGRULANAMAZ.

Transport arayuzu (SerialManager ile ayni):
    is_open, open(dev, baud), close(), write_async(text, done_cb), list_ports()
    on_line(str), on_disconnect(str)  -- disaridan atanir
"""

STROKE_MAX_MM = 600


class SimMotor:
    __slots__ = ("state", "pos", "target", "start_at", "last_t", "zero_at")

    def __init__(self):
        self.state = "I"       # I M H S F (firmware harfleri)
        self.pos = 0.0         # encoder sayacindan hesaplanan mm (raporlanan)
        self.target = 0.0
        self.start_at = 0.0
        self.last_t = 0.0
        self.zero_at = None    # home: sifira ulasma ani


class SimModule:
    def __init__(self, sid):
        self.sid = sid
        self.motors = [SimMotor() for _ in range(9)]
        self.u2_down = False   # U2 cevap vermiyor -> U2ERR / *ERR
        self.u3_down = False   # modul hic cevap vermiyor (enerjisiz / kablo)


class SimBridge:
    def __init__(self, timers, modules=(1, 2, 3, 4), speed_mm_s=4.0,
                 variant="v5f411", reply_delay_ms=70, u2_timeout_ms=1500,
                 stagger_ms=1000, home_grace_s=3.0, home_settle_s=1.2,
                 write_delay_ms=2):
        self._ts = timers
        self.mods = {int(s): SimModule(int(s)) for s in modules}
        self.speed = float(speed_mm_s)
        self.variant = variant                 # "v5f411" veya "v5.1"
        self.reply_delay_ms = reply_delay_ms
        self.u2_timeout_ms = u2_timeout_ms
        self.stagger_ms = stagger_ms
        self.home_grace_s = home_grace_s
        self.home_settle_s = home_settle_s
        self.write_delay_ms = write_delay_ms

        self.on_line = None
        self.on_disconnect = None
        self._open = False

        # --- hata enjeksiyonu ---
        self.write_ok = True          # bool veya callable(cmd) -> bool
        self.reply_filter = None      # callable(cmd, line) -> line | None (None = kayip)
        self.stat_override = {}       # sid -> "S0,S0,..." (sabit token dizisi)
        self.extra_delay_ms = {}      # sid -> ek cevap gecikmesi

        # --- gozlem ---
        self.sent = []                # gonderilen komutlar (yazma basarili)
        self.attempted = []           # yazma denenen her komut
        self.busy_until = 0.0         # hatta bekleyen cevabin bitis zamani
        self.overlaps = 0             # cevap beklenirken yeni yazma (hat cakismasi riski)

    # ================= transport arayuzu =================
    @staticmethod
    def list_ports():
        return [{"device": "SIM", "description": "Simulasyon koprusu"}]

    @property
    def is_open(self):
        return self._open

    def open(self, device="SIM", baud=115200):
        self._open = True
        return True

    def close(self):
        self._open = False

    def write_async(self, text, done_cb):
        """Yazmayi zamanlayici uzerinden tamamla (gercek yazici thread'i gibi)."""
        self._ts.call_later(self.write_delay_ms,
                            lambda: done_cb(self._write(text)))

    def write_line(self, text):
        """Senkron yazma (eski arayuz)."""
        return self._write(text)

    # ================= hata enjeksiyonu =================
    def inject(self, line, delay_ms=0):
        """Rastgele satir enjekte et (eski / sira disi / bozuk cevap)."""
        self._ts.call_later(delay_ms, lambda: self._emit(line))

    def disconnect(self, reason="USB baglantisi koptu"):
        self._open = False
        if self.on_disconnect:
            self.on_disconnect(reason)

    def fail_motor(self, sid, motor):
        self._update()
        m = self.mods[sid].motors[motor - 1]
        m.state = "F"

    def reset_module(self, sid):
        """Slave yeniden baslar: sayaclar sifir, durum IDLE (fiziksel konum bilinmez)."""
        for m in self.mods[sid].motors:
            m.state, m.pos, m.target, m.zero_at = "I", 0.0, 0.0, None

    def disturb(self, sid, motor, delta_mm):
        """Yerlesmis motoru dis etkiyle kaydir (firmware REARM -> MOVING)."""
        self._update()
        m = self.mods[sid].motors[motor - 1]
        m.pos += delta_mm
        if m.state == "S" and abs(m.target - m.pos) > 0.75:
            m.state = "M"
            m.start_at = self._ts.now()
            m.last_t = m.start_at

    def set_state(self, sid, tokens):
        """Test kurulumu: modulun motorlarini dogrudan ayarla (orn. ["S300"]*9)."""
        for m, tok in zip(self.mods[sid].motors, tokens):
            m.state = tok[0]
            m.pos = float(tok[1:])
            m.target = m.pos
            m.last_t = self._ts.now()

    def positions(self, sid):
        self._update()
        return [m.pos for m in self.mods[sid].motors]

    def states(self, sid):
        self._update()
        return "".join(m.state for m in self.mods[sid].motors)

    # ================= ic isleyis =================
    def _emit(self, line):
        if self._open and self.on_line:
            self.on_line(line)

    def _write(self, text):
        cmd = text.strip()
        self.attempted.append(cmd)
        ok = self.write_ok(cmd) if callable(self.write_ok) else bool(self.write_ok)
        if not ok or not self._open:
            return False
        now = self._ts.now()
        if now < self.busy_until:
            self.overlaps += 1
        self.sent.append(cmd)
        self._handle(cmd)
        return True

    def _reply(self, cmd, sid, line, delay_ms):
        if self.reply_filter is not None:
            line = self.reply_filter(cmd, line)
            if line is None:
                return
        delay_ms += self.extra_delay_ms.get(sid, 0)
        done = self._ts.now() + delay_ms / 1000.0
        self.busy_until = max(self.busy_until, done)
        self._ts.call_later(delay_ms, lambda: self._emit(line))

    def _u2_motors(self):
        return range(5, 10) if self.variant == "v5f411" else range(1, 6)

    def _update(self):
        now = self._ts.now()
        for mod in self.mods.values():
            for m in mod.motors:
                self._step(m, now)

    def _step(self, m, now):
        if m.state not in ("M", "H"):
            m.last_t = now
            return
        if now <= m.start_at:
            m.last_t = now
            return
        t0 = max(m.last_t, m.start_at)
        dist = self.speed * max(0.0, now - t0)
        if m.state == "M":
            err = m.target - m.pos
            if abs(err) <= dist + 0.25:
                m.pos = m.target
                m.state = "S"
            else:
                m.pos += dist if err > 0 else -dist
        else:  # H
            if m.pos > 0.0:
                if m.pos <= dist:
                    reach = t0 + m.pos / self.speed
                    m.pos = 0.0
                    m.zero_at = reach
                else:
                    m.pos -= dist
            if m.pos <= 0.0 and m.zero_at is None:
                m.zero_at = t0
            if m.zero_at is not None:
                settled_at = max(m.zero_at + self.home_settle_s,
                                 m.start_at + self.home_grace_s + self.home_settle_s)
                if now >= settled_at:
                    m.pos, m.target, m.state = 0.0, 0.0, "S"
        m.last_t = now

    def _schedule_move(self, m, mm, slot):
        now = self._ts.now()
        m.target = float(max(0, min(STROKE_MAX_MM, mm)))
        m.start_at = now + slot * self.stagger_ms / 1000.0
        m.last_t = now
        m.state = "M"

    def _schedule_home(self, m, slot):
        now = self._ts.now()
        m.start_at = now + slot * self.stagger_ms / 1000.0
        m.last_t = now
        m.zero_at = None
        m.state = "H"

    def _tokens(self, mod):
        toks = []
        for m in mod.motors:
            toks.append(f"{m.state}{int(m.pos)}")
        return toks

    def _handle(self, cmd):
        self._update()
        parts = cmd.split(":")
        verb = parts[0]
        if len(parts) < 2:
            return
        try:
            sid = int(parts[1])
        except ValueError:
            return
        mod = self.mods.get(sid)
        if mod is None or mod.u3_down:
            return
        d = self.reply_delay_ms
        u2_ms = self.u2_timeout_ms if mod.u2_down else 15
        u2set = set(self._u2_motors())

        if verb == "PING":
            self._reply(cmd, sid, f"PONG:{sid:02d}", d)
        elif verb == "GETID":
            self._reply(cmd, sid, f"IDVAL:{sid:02d}", d)
        elif verb == "STAT":
            if sid in self.stat_override:
                body = self.stat_override[sid]
                self._reply(cmd, sid, f"STAT:{sid:02d}:{body}", d + u2_ms)
                return
            toks = self._tokens(mod)
            if mod.u2_down:
                loc = [t for i, t in enumerate(toks) if (i + 1) not in u2set]
                body = (",".join(loc) + ",U2ERR") if self.variant == "v5f411" \
                    else ("U2ERR," + ",".join(loc))
            else:
                body = ",".join(toks)
            self._reply(cmd, sid, f"STAT:{sid:02d}:{body}", d + u2_ms)
        elif verb == "MOV" and len(parts) >= 4:
            try:
                motor, val = int(parts[2]), int(parts[3])
            except ValueError:
                return
            if val < 0 or val > STROKE_MAX_MM or not 1 <= motor <= 9:
                return
            ok = True
            if motor in u2set and mod.u2_down:
                ok = False
            else:
                self._schedule_move(mod.motors[motor - 1], val, 0)
            tag = "MOVOK" if ok else "MOVERR"
            self._reply(cmd, sid, f"{tag}:{sid:02d}:{motor:02d}:{val}",
                        d + (u2_ms if motor in u2set else 0))
        elif verb == "ALL" and len(parts) >= 3:
            try:
                val = int(parts[2])
            except ValueError:
                return
            if val < 0 or val > STROKE_MAX_MM:
                return
            for i, m in enumerate(mod.motors):
                if (i + 1) in u2set and mod.u2_down:
                    continue        # U2 komutu almadi
                self._schedule_move(m, val, i)   # U3 yerelleri yine zamanlar
            tag = "ALLERR" if mod.u2_down else "ALLOK"
            self._reply(cmd, sid, f"{tag}:{sid:02d}:{val}", d + u2_ms)
        elif verb == "ARR":
            vals = parts[2:]
            if len(vals) != 9:
                self._reply(cmd, sid, "ARRERR:FMT", d)
                return
            try:
                nums = [int(v) for v in vals]
            except ValueError:
                nums = [int(v) if v.lstrip("-").isdigit() else 0 for v in vals]
            for i, m in enumerate(mod.motors):
                if (i + 1) in u2set and mod.u2_down:
                    continue
                self._schedule_move(m, nums[i], i)
            tag = "ARRERR" if mod.u2_down else "ARROK"
            self._reply(cmd, sid, f"{tag}:{sid:02d}", d + u2_ms)
        elif verb == "HOME":
            motor = int(parts[2]) if len(parts) >= 3 and parts[2].isdigit() else 0
            if motor == 0:
                for i, m in enumerate(mod.motors):
                    if (i + 1) in u2set and mod.u2_down:
                        continue
                    self._schedule_home(m, i)
                tag = "HOMEERR" if mod.u2_down else "HOMEOK"
                self._reply(cmd, sid, f"{tag}:{sid:02d}:00", d + u2_ms)
            elif 1 <= motor <= 9:
                ok = not (motor in u2set and mod.u2_down)
                if ok:
                    self._schedule_home(mod.motors[motor - 1], 0)
                tag = "HOMEOK" if ok else "HOMEERR"
                self._reply(cmd, sid, f"{tag}:{sid:02d}:{motor:02d}",
                            d + (u2_ms if motor in u2set else 0))
        elif verb == "GETPOS" and len(parts) >= 3:
            try:
                motor = int(parts[2])
            except ValueError:
                return
            if not 1 <= motor <= 9:
                return
            m = mod.motors[motor - 1]
            if motor in u2set and mod.u2_down:
                self._reply(cmd, sid, f"POSERR:{sid:02d}:{motor:02d}", d + u2_ms)
            else:
                self._reply(cmd, sid,
                            f"POS:{sid:02d}:{motor:02d}:{int(m.pos)}:{int(m.pos * 79.93)}", d)
        elif verb == "SETID" and len(parts) >= 3:
            # simulasyon kimligi degistirmez; yalniz cevap verir
            self._reply(cmd, sid, f"IDSET:{sid:02d}", d)
