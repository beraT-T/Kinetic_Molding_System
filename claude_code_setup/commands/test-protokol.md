---
description: Donanim test sirasini checklist olarak ureten yardimci
---
Bir slave (varsayilan 01, `$ARGUMENTS` verilirse o) icin v5.1 donanim test sirasini adim adim,
checklist halinde uret. Kullanici komutlari elle gonderecek; sen sadece sirayi ve beklenen sonucu yaz.

Sira:
1. `HOME:<id>`  -> ack `HOMEOK:<id>:00`. 9 motor 1'er sn kademeli inmeli.
2. `STAT:<id>`  -> birkac kez sorgula; tum tokenlar `S` ve mm ~0 olana kadar bekle.
   (600'den baslayan motor TAM dibe inmeli; inmiyorsa home/limit-switch sorunu.)
3. `ALL:<id>:300` -> 9 motor ~300mm'ye gitmeli. `STAT` ile dogrula (~298-300 normal).
4. `ARR:<id>:p1:..:p9` -> farkli hedefler; salinim olmadan oturmali.
5. Guvenlik: bir encoder kablosunu gevsetip ilgili motoru sur -> `STAT`'ta `F` (FAULT) gormeli.
6. mm hassasiyeti: bir motoru `MOV:<id>:<m>:<mm>` ile bilinen degere gonder, cetvelle olc,
   sapma >2-3mm ise ilgili motorun `cal` degerini kalibrasyonla ayarla.

Her adimda "beklenen" ile "gozlenen"i kullanicidan iste; sapma varsa CLAUDE.md'deki olasi
sebeplere (yon, encoder baglanti, cal, home) gore yonlendir.
