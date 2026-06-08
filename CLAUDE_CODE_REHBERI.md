# Claude Code'a Geçiş ve Projeyi Yönetme Rehberi

Bu projedeki know-how'ı Claude Code'a taşıdık. Mekanizma ve nasıl yöneteceğin aşağıda.

## 1. Know-how nasıl aktarılıyor

Claude Code her oturumda projedeki **`CLAUDE.md`** dosyasını otomatik okur — projenin kalıcı belleği.
Mimari, donanım, protokol v5.1, kalibrasyon gerçekleri, home fix, UI, git ve test döngüsü orada.
Yani yeni bir Claude Code oturumu açtığında bu bağlamı baştan biliyor olacak.

- Kök `CLAUDE.md`: her zaman yüklenir (genel, kararlı bilgi).
- Alt klasör `CLAUDE.md` (örn. `UI_v2.0_QML/CLAUDE.md`): sadece o klasördeki bir dosya okunduğunda
  yüklenir → derine inen, yola özel kuralları oraya koyabilirsin.
- Global `~/.claude/CLAUDE.md`: tüm projelerinde geçerli (kişisel tercihlerin).
- Zamanla güncel tut: yeni bir karar/bulgu olunca `CLAUDE.md`'ye ekle (Claude Code'a "bunu
  CLAUDE.md'ye yaz" diyebilirsin, ya da `/init` ile tazeleyebilirsin).

## 2. Kurulum (bir kerelik)

```bash
# Claude Code kurulumu (resmi yönergeyi izle): https://code.claude.com/docs
cd ~/Documents/GitHub/Kinetic_Molding_System
claude          # proje kökünde başlat -> CLAUDE.md otomatik okunur
```

İlk girişte istersen `/init` çalıştır: var olan `CLAUDE.md`'yi bozmaz, iyileştirme önerir.

### .gitignore düzeltmesi (önemli)
Daha önce tüm `.claude/`'yi ignore etmiştik; ama Claude Code yapılandırması (`commands`, `settings`)
`.claude/` içinde yaşar ve **commit edilmeli**. Şunu yap:

```bash
# .gitignore içindeki ".claude/" satirini sil, yerine:
echo ".claude/worktrees/" >> .gitignore
echo ".claude/settings.local.json" >> .gitignore
git add .gitignore .claude/commands .claude/settings.json 2>/dev/null
git commit -m "chore: Claude Code yapilandirmasini takibe al"
```

## 3. Projeyi yönetme akışı (kod yazmadan yönlendirme)

Sen yönetirsin, Claude Code yazar/test eder. Önerilen döngü:

1. **Plan modu (Shift+Tab):** Büyük/çok-dosyalı bir iş isteyeceksen önce plan moduna geç. Claude
   sadece okur ve bir **plan** sunar; dosya değiştiremez. Planı `Ctrl+G` ile editörde düzeltebilir,
   onaylayınca uygulamaya geçer. 3+ dosyaya dokunan her işte bunu kullan — en güvenli yöntem.
2. **Net görev ver:** "Şu davranışı ekle/düzelt, şu dosyalarda, şu protokole uy." CLAUDE.md zaten
   bağlamı verdiği için kısa tutabilirsin.
3. **Donanım-döngüsü:** Claude kodu yazar → **sen** PlatformIO ile yükler ve test edersin →
   `STAT`/seri çıktısını ve gözlemini geri verirsin → Claude iterasyon yapar. (Claude kart
   flash'layamaz; bu döngü CLAUDE.md'de de yazılı.)
4. **Gözden geçirme:** Değişiklik sonrası `/review` çalıştır — `CLAUDE.md` + `REVIEW.md`'deki
   kriterlere göre kontrol eder (pin/timer tutarlılığı, protokol uyumu, v4.4'e dokunulmaması...).
5. **Commit:** Anlamlı, gruplu commit'ler iste; sen onayla.

## 4. Faydalı araçlar

- **Slash komutları** (`.claude/commands/*.md`): tekrarlayan işler için kısayol. Hazır örnekler
  `claude_code_setup/commands/` altında — Claude Code'un görmesi için bir kerelik kopyala:

  ```bash
  mkdir -p .claude/commands
  cp claude_code_setup/commands/*.md .claude/commands/
  ```

  Gelen komutlar:
  - `/flash <U2|U3|cal-u2|cal-u3>` — ilgili PlatformIO projesini derleyip yükleme adımları.
  - `/test-protokol [id]` — donanım test sırası (HOME → STAT → ARR ...) checklist olarak.
  - `/durum` — son commit'ler + açık işlerden kısa durum özeti.

  Yeni komut: `.claude/commands/ad.md` aç, içine prompt yaz; `$ARGUMENTS` ile parametre alır.
- **Subagent'lar** (`~/.claude/agents/ad.md`): odaklı uzman ajanlar. Örn. bir "firmware-review"
  ajanı (sadece okuma + pin/protokol kontrolü) ya da "qml-lint" ajanı. Frontmatter: `name`,
  `description` ("şu durumda kullan"), opsiyonel `tools`, `model`.
- **GitHub PR review:** Claude Code GitHub Actions ile her PR'da otomatik review (CLAUDE.md +
  REVIEW.md'yi kullanır). İleride `ui_v2.0` → `main` birleştirmelerinde işine yarar.
- **`/memory`:** belleği (CLAUDE.md / auto-memory) görüntüle/düzenle.

## 5. Bu proje için pratik öneriler

- Firmware işlerinde **her zaman** önce plan modu + sonra donanımda test; v4.4 klasörlerine
  dokundurma (CLAUDE.md'de yazılı, REVIEW.md'de denetlenir).
- UI işlerini `ui_v2.0` branch'inde yürüt: `git checkout ui_v2.0`.
- Yeni bir bulgu/karar (örn. bir motorun farklı `cal`'ı, yeni protokol komutu) çıkınca Claude'a
  "bunu CLAUDE.md'ye işle" de — bellek güncel kalsın.
- Büyük bir özellik öncesi: "plan modunda bana yaklaşım çıkar" → planı oku/düzelt → onayla.

Resmi dokümanlar: https://code.claude.com/docs  (memory, sub-agents, permission-modes, code-review,
github-actions başlıkları bu akışın tamamını anlatır.)
```
