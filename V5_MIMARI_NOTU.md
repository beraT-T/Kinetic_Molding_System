# v5.1 — Mimari + Protokol Notu

İki yeni proje (mevcut v4.4'e dokunulmadı):

- `Adaptif_kalip_v5_U3_4motor/` — U3 (yönetici): RS485 + motor 6-9 + U2'ye dağıtım + flash slave ID
- `Adaptif_kalip_v5_U2_5motor/` — U2 (işçi): sadece USART2 + motor 1-5

## Home bug'ı — sebep ve çözüm

**Sebep:** Hız ~4 mm/s, encoder 79.93 puls/mm → 600 mm tam iniş = **~150 saniye**. Eski `HOME_MAX_MS = 70000` (70 sn) tam stroktan kısaydı; motor dibe inemeden süre dolup encoder sıfırlanıyor ve yanlış `HOMEOK` dönüyordu. Üstüne home bloklayıcı ve sırayla çalışıyordu.

**Çözüm:**

- `HOME_MAX_MS = 200000` (200 sn) — tam strok + pay.
- Home artık **non-blocking** bir durum (`HOMING`). Entegre limit switch'in motoru fiziksel 0'da kestiğinde encoder değişmez; kalkış payı (`HOME_GRACE_MS = 3000`) sonrası encoder `HOME_SETTLE_MS = 1200` ms değişmezse "fiziksel 0'a varıldı" kabul edilip sayaç sıfırlanır. Demeraj gecikmesi grace'i aşsa bile erken bitiş olmaz.
- 9 motor aynı anda (1 sn kademeli) home olur, sırayla değil.

## Senaryoya uygun mimari

Host (PC/Pi) → F103 köprüsü → U3 → U2. Bir slave 9 motor (1-5 U2'de, 6-9 U3'te). STL'den gelen array bir slave'e ulaştığında 9 motor **1'er saniye kademeli başlayıp eş zamanlı** hedefe gider (silikon yekpare). 150 sn'lik harekette 1 sn kademe ihmal edilebilir → pratikte hepsi birlikte.

Kademe slotları: motor 1-5 → slot 0..4 (U2), motor 6-9 → slot 5..8 (U3). Yani 9 motor düzgün 1'er sn arayla kalkar.

**Async model:** Hareket 150 sn sürdüğü için komutlar **hemen ack döner** ("başladı" anlamında), motorlar arka planda gider. Host bitişi `STAT` komutuyla **poll eder**. Host (Pi/Python) tarafını buna göre güncellemen gerekir.

## RS485 komut seti (host → U3)

| Komut | Açıklama | Cevap |
|-------|----------|-------|
| `PING:id` | haberleşme testi | `PONG:id` |
| `GETID:id` / `SETID:id:yeni` | ID oku / yaz (flash) | `IDVAL`/`IDSET` |
| `MOV:id:motor:mm` | tek motor (test) | `MOVOK:id:motor:mm` (başladı) |
| `ALL:id:mm` | 9 motor aynı hedef, kademeli | `ALLOK:id:mm` |
| `ARR:id:p1:..:p9` | 9 motora ayrı hedef (STL array) | `ARROK:id` |
| `HOME:id` | 9 motor home, kademeli | `HOMEOK:id:00` |
| `HOME:id:motor` | tek motor home (test) | `HOMEOK:id:motor` |
| `GETPOS:id:motor` | tek motor pozisyon | `POS:id:motor:mm:puls` |
| `STAT:id` | 9 motorun durum+pozisyonu | aşağıda |

`STAT` cevabı: `STAT:id:<t1>,<t2>,...,<t9>` — her token `<durum harfi><mm>`. Durum harfleri:

- `I` = boşta (IDLE)
- `M` = hedefe gidiyor (MOVING)
- `H` = home yapıyor (HOMING)
- `S` = yerine oturdu (SETTLED) ← hepsi `S` ise hareket tamam
- `F` = arıza (stall/timeout — encoder kopuk ya da takıldı)

Örnek: `STAT:01:S0,S120,M300,H0,S50,S200,M450,S0,S600`

**Host akışı (öneri):** `ARR`/`HOME` gönder → ack al → her 1-2 sn'de bir `STAT:id` gönder → tüm tokenlar `S` olunca o slave bitti. Tüm slave'ler bitince yüzey şeklini aldı.

## v4.4'e göre özet değişiklikler

1. Tek `Motor` struct + durum makinesi (IDLE/MOVING/HOMING/SETTLED/FAULT), tamamen non-blocking.
2. **Histerezis** (STOP_BAND=20, REARM_BAND=60 puls) → hedef etrafında salınım biter.
3. **Stall/encoder-kopma koruması** (MOV'da 1.5 sn hareketsizlik → FAULT).
4. **Encoder giriş filtresi** (hall gürültüsü).
5. **Home non-blocking + doğru timeout + limit-switch mantığı** (ana bug fix).
6. **1 sn kademeli eş zamanlı** çoklu hareket; yeni `ARR`/`HOME-all`/`STAT`.
7. U2 beklenirken U3 yerel motorları servis edilir (kilitlenme yok).

## Ayarlanabilir sabitler (dosya başı)

| Sabit | Değer | Not |
|-------|-------|-----|
| `CAL_HARDWARE` | 79.93 | puls/mm — cetvelle doğrula |
| `STOP_BAND` | 20 | hedef toleransı (~0.25 mm) |
| `REARM_BAND` | 60 | histerezis |
| `STAGGER_MS` | 1000 | motorlar arası kalkış |
| `HOME_MAX_MS` | 200000 | home timeout |
| `HOME_GRACE_MS` | 3000 | kalkış payı |
| `HOME_SETTLE_MS` | 1200 | "encoder durdu = 0" süresi |

## Test sırası

1. U2 ve U3'ü flash'la.
2. `HOME:01` gönder → `HOMEOK:01:00` gelmeli; 9 motor 1'er sn arayla inmeli. `STAT:01` ile izle, hepsi `S` ve mm≈0 olmalı. **600'den başlayan motor artık tam dibe inmeli.**
3. `ARR:01:100:150:200:...:9 değer` ile farklı hedefler dene; salınımsız oturmalılar.
4. mm hassasiyeti: bir motoru `MOV` ile bilinen değere gönder, cetvelle ölç, sapmayı not et → gerekirse `STOP_BAND`/`CAL_HARDWARE` ince ayar.
5. Bir encoder kablosunu gevşetip `STAT`'ta o motorun `F` (FAULT) olduğunu doğrula.
