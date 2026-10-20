# Depo haritası

> Hazırlandı: 2026-10-19, dal `claude/ui-v2.2-bulgu-duzeltme` (taban `ui_v2.2` @ `2c5cf2c`).
> Kaynak: klasör içerikleri, `platformio.ini` başlıkları, kaynak dosya başlıkları, `git ls-files`,
> `CLAUDE.md`. Kanıt bulunamayan alanlar **bilinmiyor** olarak yazıldı. Hiçbir dosya taşınmadı/silinmedi.
> "Git" sütunu: **izlenen** = `git ls-files` içinde; **takip edilmiyor** = çalışma ağacında var, hiç commit
> edilmemiş (bu dalda da eklenmedi).

## Firmware (STM32, PlatformIO)

| Klasör | Amaç | Kart / env | Firmware sürümü | Giriş noktası | Durum | Git | Test türü |
|---|---|---|---|---|---|---|---|
| `Adaptif_kalip_v5f411_U3_4motor/` | Modül yöneticisi: RS485 + yerel motor 1–4 + flash ID | F411CEU6 / `blackpill_f411ce` | v5f411 (v5.1 portu) | `src/main.cpp` | Güncel (F411 kartlar; `CLAUDE.md`) | **takip edilmiyor** | Donanımda elle (`V5F411_NOTU.md`) |
| `Adaptif_kalip_v5f411_U2_5motor/` | Modül işçisi: USART2 + motor 5–9 | F411CEU6 / `blackpill_f411ce` | v5f411 | `src/main.cpp` | Güncel (F411 kartlar) | **takip edilmiyor** | Donanımda elle |
| `Adaptif_kalip_v5_U3_4motor/` | Modül yöneticisi: RS485 + yerel motor 6–9 + flash ID | F401RC / `genericSTM32F401RC` | v5.1 | `src/main.cpp` | Güncel (F401 kartlar) | izlenen | Donanımda elle (`V5_MIMARI_NOTU.md` "Test sırası") |
| `Adaptif_kalip_v5_U2_5motor/` | Modül işçisi: USART2 + motor 1–5 | F401RC / `genericSTM32F401RC` | v5.1 | `src/main.cpp` | Güncel (F401 kartlar) | izlenen | Donanımda elle |
| `Adaptif_kalip_v4_slave1_rs485_4motor/` | Eski modül yöneticisi (bloklayıcı, bang-bang) | F401RC | v4.4 | `src/main.cpp` | Eski, çalışan temel — **DOKUNMA** | izlenen | `test/` yalnız PlatformIO şablon README; otomatik test yok |
| `Adaptif_kalip_v4_slave2_5motor/` | Eski modül işçisi | F401RC | v4.4 | `src/main.cpp` | Eski — **DOKUNMA** | izlenen | Şablon README |
| `Adaptif_Kalip_blue_pill_rs485_test/` | Host ↔ RS485 şeffaf köprü (RS485 USART2, PA2/PA3) | F103C8 / `bluepill_f103c8` | v4.x köprü | `src/main.cpp` | Eski köprü (v4.4 seti, **DOKUNMA**) | izlenen (alt klasör `Tester_UI/ui_v2.0.py` çalışma ağacında **silinmiş**, commit edilmedi — aşağıya bakın) | Şablon README |
| `Adaptif_Kalip_blue_pill_rs485_bridge_usart1/` | Aynı köprü, RS485 USART1'e (PA9/PA10, 5 V toleranslı) taşınmış | F103C8 / `bluepill_f103c8` | v4.2 (USART1) | `src/main.cpp` | Yeni/düzeltme sürümü; sahada hangisinin yüklü olduğu **bilinmiyor** (`RS485_DENETIM_NOTU.md` "zaten USART1'e taşındı" diyor) | **takip edilmiyor** | Donanımda elle |
| `Calibration_U2_5motor/` | Yön/puls-mm ölçümü (USB seri menü `h,t,f,r,z,l,s`) | F401RC | araç (v5.1 numaralandırma 1–5) | `src/main.cpp` | Araç; F411 kartta kullanılabilirliği **bilinmiyor** (doğrulanmadı) | izlenen | Donanımda elle (`KALIBRASYON_REHBERI.md`) |
| `Calibration_U3_4motor/` | Aynı, U3 (motor 6–9) | F401RC | araç | `src/main.cpp` | Araç; F411'de **bilinmiyor** | izlenen | Donanımda elle |
| `RS485_Modul_Tester_f411/` | RS485 alıcı-verici (MAX485) test cihazı, ekranlı; env `hizli` / `soak` | F411CEU6 / `blackpill_f411ce` | araç | `src/hizli_test.cpp`, `src/soak_test.cpp` (+ `main_eski_birlesik.cpp.txt` eski) | Deneysel/araç | **takip edilmiyor** | Donanımda elle |

## Arayüzler ve araçlar (host)

| Klasör | Amaç | Giriş noktası | Durum | Git | Test türü |
|---|---|---|---|---|---|
| `UI_v2.0_QML/` | PySide6 + QML HMI (Pi kiosk), protokol v5.1 | `main.py` (`--fullscreen`, `--demo`), `run.sh`, `deploy_pi.sh` | **Güncel.** Kiosk `ui_v2.0` dalında; v2.2 ve bu düzeltmeler `ui_v2.2` / `claude/ui-v2.2-bulgu-duzeltme` (deploy edilmedi) | izlenen (`docs/screenshots/` takip edilmiyor) | Donanımsız `unittest` (sahte köprü + sahte saat), gerçek QML etkileşimi (offscreen), `loop://` seri, sahte ssh deploy testi |
| `Web_UI/` | Eski Flask + React/Vite + Socket.IO arayüzü | `backend/app.py`, `frontend/` (`npm run dev`) | Eski (Pi'de kasıyordu) | izlenen | Otomatik test yok. `test_mov.py`, `test.master.py` = **fiziksel donanım araçları** (COM3'e PING/MOV yazar); artık yalnız `--donanim` ile çalışır, içe aktarılınca hiçbir şey yapmaz |
| `Tester_UI/` (kök) | customtkinter + pyserial "Kinetik Yüzey Kontrol Paneli v2.0" — SETID/GETID/PING/MOV dahil elle komut aracı; **hareket kilidi yok** | `ui_v2.0.py` | Eski araç | **takip edilmiyor** (+ yerel `venv/`) | Yok |
| `hakem_sunumu/` | Hakem sunumu için protokol/pozisyonlama simülasyonları ve figürler | `sim_protokol.py`, `sim_pozisyonlama.py` | Belge/analiz | **takip edilmiyor** | Yok (simülasyon çıktısı) |
| `makale/` | Makale taslağı (md/tex/bib, Overleaf zip) | — | Belge | **takip edilmiyor** | — |
| `photos/` | 212 fotoğraf | — | Belge | **takip edilmiyor** | — |
| `claude_code_setup/` | Claude Code komut şablonları (`durum.md`, `flash.md`, `test-protokol.md`) | — | Araç/belge | izlenen | — |
| `venv/` (kök) | Yerel Python ortamı | — | Ortam | `.gitignore` | — |
| `.claude/` | Claude Code yapılandırması; `worktrees/determined-nightingale` = ayrı worktree (`claude/determined-nightingale` @ `47b71e0`) | — | İç içe worktree — gereksiz sayılmamalı | `worktrees/`, `settings.local.json` ignore; `settings.local.json.yedek-v22-oncesi` takip edilmiyor | — |

## Kök belgeler

| Dosya | İçerik | Git |
|---|---|---|
| `CLAUDE.md` | Projenin kalıcı belleği (tek kaynak) | izlenen |
| `AGENTS.md` | `CLAUDE.md`'nin Codex eşi (2026-10-19'da eşitlendi) | **takip edilmiyor** |
| `README.md` | Genel tanıtım | izlenen |
| `REVIEW.md` | Kod inceleme kriterleri | izlenen |
| `CLAUDE_CODE_REHBERI.md` | Claude Code kurulum/yönetim rehberi | izlenen |
| `V5_MIMARI_NOTU.md` | v5.1 mimari + protokol + test sırası | izlenen |
| `KALIBRASYON_REHBERI.md` | Kalibrasyon aracı kılavuzu | izlenen |
| `V5F411_NOTU.md` | F411 portu farkları, soket sırası numaralandırması | **takip edilmiyor** |
| `RS485_DENETIM_NOTU.md` | RS-485 hattı eleştirel denetimi (ölçüm planı) | **takip edilmiyor** |
| `PMS_GOREVLERI.md` | PMS görev aktarımı | **takip edilmiyor** |
| `REPO_HARITASI.md`, `ARSIVLEME_ONERISI.md` | Bu harita ve arşivleme önerisi | izlenen (bu dal) |

## Takip edilmeyen önemli dosyalar (açık liste — topluca commit'e EKLENMEDİ)

Kullanıcının kararı gerekir; her biri ayrı, gözden geçirilmiş commit ile eklenmeli:

1. `Adaptif_kalip_v5f411_U3_4motor/` ve `Adaptif_kalip_v5f411_U2_5motor/` — **F411 kartların güncel
   firmware'i** Git'te yok. `CLAUDE.md`, `README.md` ve F401 `platformio.ini` dosyaları bunlara
   yönlendiriyor. (Bu turda yalnız `platformio.ini` başlık yorumları düzeltildi: U3 = motor 1–4,
   U2 = motor 5–9; firmware koduna dokunulmadı.)
2. `Adaptif_Kalip_blue_pill_rs485_bridge_usart1/` — USART1 köprüsü (5 V tolerans düzeltmesi).
3. `RS485_Modul_Tester_f411/` — RS-485 test cihazı firmware'i.
4. `AGENTS.md`, `V5F411_NOTU.md`, `RS485_DENETIM_NOTU.md`, `PMS_GOREVLERI.md` — belgeler
   (`V5F411_NOTU.md` tablosundaki motor aralıkları bu turda düzeltildi).
5. `Tester_UI/ui_v2.0.py` — taşınmış araç (aşağıya bakın).
6. `UI_v2.0_QML/docs/screenshots/` — v2.0 dönemi ekran görüntüleri.
7. `hakem_sunumu/`, `makale/`, `photos/` — belge/veri (büyük; ayrı karar).

## Tester_UI taşıma durumu (doğrulandı)

- İzlenen eski yol `Adaptif_Kalip_blue_pill_rs485_test/Tester_UI/ui_v2.0.py` çalışma ağacında
  **silinmiş** (` D`, commit edilmemiş); kökte `Tester_UI/ui_v2.0.py` **takip edilmiyor**.
- İki dosya **birebir aynı**: `HEAD` içeriği ile kökteki dosya bayt bayt eşit (`cmp` farksız),
  SHA-256 `7cea21fb152d9fb87a402703686f3a884c1b0732e42b412766ad31b377f817aa` (18 950 bayt).
- Yani taşıma yarım: Git açısından dosya "silindi", yeni yeri eklenmedi. Bu dalda **dokunulmadı**
  (kullanıcı değişikliği). Tamamlamak için tek commit'te `git rm` (eski yol) + `git add Tester_UI/ui_v2.0.py`
  yeterlidir (`git mv` eşdeğeri); `Tester_UI/venv/` eklenmemeli (kök `.gitignore` `venv/` kapsıyor).

## Testler: hangisi nerede, hangisi donanıma dokunur

| Konum | Tür | Donanıma dokunur mu |
|---|---|---|
| `UI_v2.0_QML/tests/` | `unittest` (sahte köprü, sahte saat, `loop://`, offscreen QML, sahte ssh) | **Hayır** (`test_repo_safety.py` denetler). Çalıştırma: `cd UI_v2.0_QML && QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests` |
| `Web_UI/test_mov.py`, `Web_UI/test.master.py` | Elle donanım aracı | **Evet** (COM3; `--donanim` gerekir) |
| `Tester_UI/ui_v2.0.py` | Elle GUI aracı | **Evet** (port seçilip bağlanınca; SETID dahil) |
| Firmware `test/` klasörleri | PlatformIO şablon README | — (test yok) |
| `Calibration_*`, `RS485_Modul_Tester_f411` | Donanım test firmware'i | **Evet** (karta yüklenir) |

Repo kökünde `pytest` / kör `unittest discover` **çalıştırmayın**.
