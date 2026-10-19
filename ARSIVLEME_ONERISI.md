# Arşivleme önerisi (UYGULANMADI)

> 2026-10-19. Yalnız öneri: bu turda hiçbir dosya silinmedi, taşınmadı. Her madde kullanıcı kararı
> gerektirir. Donanım varyantları ve iç içe worktree "gereksiz" **sayılmadı** (F401/F411 iki kart
> ailesi sahada birlikte bulunabilir; worktree başka bir oturumun çalışma alanıdır).

## İlke

- Silme yok; arşiv = ayrı klasör (`arsiv/`) veya ayrı Git dalı/etiketi + README'de tek satır işaret.
- Önce **etiket**: taşımadan önce mevcut durumu `git tag arsiv-oncesi-<tarih>` ile işaretle.
- Bir dosyayı arşivlemeden önce: başka belge/kod ona yönlendiriyor mu? (`grep -rn <ad>`)
- Firmware'de "çalışan temel" (v4.4) **arşivlenmez**, yerinde kalır (CLAUDE.md: DOKUNMA).

## Aday maddeler

| Madde | Öneri | Gerekçe | Risk / ön koşul |
|---|---|---|---|
| `Tester_UI/ui_v2.0.py` (kök, takip edilmiyor) + silinmiş eski yolu | Taşımayı tamamla (tek commit: eski yolu `git rm`, yeni yolu `git add`) **veya** eski yolu geri getirip kökteki kopyayı sil | Şu an Git'te "silindi" görünüyor; iki kopya birebir aynı | Araç SETID dahil hareket kilitsiz komut gönderir: README'ye "fiziksel donanım aracı" uyarısı eklenmeli |
| `Web_UI/` | `arsiv/Web_UI/` veya `legacy-web-ui` dalı | UI_v2.0_QML yerini aldı (Pi'de kasıyordu) | `test_mov.py`/`test.master.py` artık `--donanim` ister; arşivde de öyle kalmalı |
| `RS485_Modul_Tester_f411/main_eski_birlesik.cpp.txt` | Aynı klasörde `eski/` alt klasörü | İki yeni program (`hizli`, `soak`) ile değiştirilmiş görünüyor | Derlemeye girmiyor (`.txt`); bilinmeyen içerik farkı olabilir |
| `UI_v2.0_QML/docs/screenshots/` (v2.0 dönemi) | `docs/v2.0/ekran/` | v2.2 ekranları `docs/v2.2/ekran*`; ayrım netleşir | Takip edilmiyor; önce eklenip eklenmeyeceği seçilmeli |
| `UI_v2.0_QML/docs/v2.2/ekran/` (gece turu) | Olduğu gibi kalsın ("önce" kanıtı); güncel olanlar `ekran_duzeltme/` | İnceleme bulgusunu (Bakım ekranı taşması) belgeliyor | — |
| `photos/` (212 dosya), `makale/`, `hakem_sunumu/` | Ayrı depo veya Git LFS; ana depoya ham fotoğraf eklenmemeli | Boyut; kod deposu değil | Makale/hakem dosyalarının kaynağı korunmalı |
| `Adaptif_Kalip_blue_pill_rs485_test/` ↔ `..._bridge_usart1/` | Arşivleme **değil**: sahadaki köprünün hangisi olduğu belirlenip README'ye yazılmalı | Sahada hangisinin yüklü olduğu bilinmiyor | Donanım bilgisi gerekir |
| `Calibration_*` (F401) | Arşivleme değil; F411 kartlar için port gerekip gerekmediği karar | F411'de kullanılabilirliği doğrulanmadı | — |
| `.claude/settings.local.json.yedek-v22-oncesi` | Kullanıcı yedeği; dokunulmaz | v2.2 gece turu öncesi yedek | — |
| `.claude/worktrees/determined-nightingale` | Dokunulmaz (ayrı oturum/dal) | İç içe worktree | Silmek o dalın çalışma alanını kaybettirir |

## Eklenmesi önerilen (arşiv değil, takip)

`Adaptif_kalip_v5f411_*`, `Adaptif_Kalip_blue_pill_rs485_bridge_usart1`, `RS485_Modul_Tester_f411`,
`V5F411_NOTU.md`, `RS485_DENETIM_NOTU.md`, `AGENTS.md` — güncel/önemli ama Git'te yok
(`REPO_HARITASI.md` → "Takip edilmeyen önemli dosyalar"). Her biri ayrı commit ile, içerik
gözden geçirilerek eklenmeli; `git add -A` kullanılmamalı.
