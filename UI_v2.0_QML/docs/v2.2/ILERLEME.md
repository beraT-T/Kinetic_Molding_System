# v2.2 İlerleme

> Oturum yeniden başlarsa veya bağlam sıkıştırılırsa: önce bu dosya, sonra `KARARLAR.md`,
> sonra `GOREV.md`. Kaldığın yerden devam et.

## SABAH ÖZETİ
_(gece sonunda doldurulacak — biçim: GOREV.md bölüm E)_

## Durum
- Branch: `ui_v2.2` (henüz açılmadı)
- Aktif faz: —
- Son commit: —

## Plan
_(Faz 0 başında yazılacak)_

## Günlük
<!-- tarih saat — faz — ne yapıldı / sonuç -->

## ENGELLENDİ / STASH
<!-- ENGELLENDİ: <komut> — <sebep>  |  STASH: v22-wip-<konu> — <dosyalar> — <neden> -->

## Kapsama tablosu

| İnceleme maddesi | Faz | Yapılan | Test | Kalan / gerekçe |
|---|---|---|---|---|
| 1 Eski STAT → başarı | 1b | | | protokol kimliği yok → öneri |
| 2 STOP yok / metin | 1e, 5 | | | donanım değerlendirmesi |
| 3 Tester/terminal kilit aşımı | 1a | | | çift dokunma → panel |
| 4 Arızada gözetim bırakılıyor | 1c | | | |
| 5 USB/slave reset | 2b, 2c, 2d | | | gerçek USB → donanım |
| 6 Kısmi STAT / U2ERR | 2f | | | U3 davranışı → öneri |
| 7 Alarm onayı siliyor | 2e | | | |
| 8 Eski dispatch callback | 1d | | | |
| 9 Atomik STL + şeffaflık | 3a, 3b | | | komşu fark limiti → kullanıcı |
| 10 Hedef/ölçüm/modül karışması | 3c | | | canlı ölçüm haritası → öneri |
| 11 Tek seri yönetici | 2a | | | hat simülasyonu |
| 12 HOME referansı kanıt değil | metin + 5 | | | firmware/donanım |
| 13 Eksik modül / referanssız test / Tester aralığı | 2g | | | config değerleri → kullanıcı |
| 14 UI thread yazma / sınırsız tampon | 2a | | | Pi ölçümü yapılmadı |
| Metin tablosu (16 satır) | 4a | | | |
| Ek dil/akış (7 madde) | 4b | | | |
| Dokunmatik/görsel (6 madde) | 4c | | | panel |
| Kabul matrisi (14 satır) | 0, 5 | | | |
| Demo yetersizliği | 0, 4b | | | |
