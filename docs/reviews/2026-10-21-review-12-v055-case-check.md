# Tunnel-navigation repo incelemesi, 5. tur: v0.5.5, case metniyle karşılaştırma, Pages

Repo: https://github.com/alpermutlu2/Tunnel-navigation
İncelenen sürüm: `bc01c5b` (v0.5.5, 21 Ekim 12:30)
Önceki tur: v0.5.4 (`59e52bb`), `tunnel-navigation-inceleme-v4.md`
İnceleme: 21 Ekim 2026, 13:15–13:35

Bu belge üç şeyi kapsar: (A) v0.5.5'in doğrulanması (IMU sınıfı tespiti, MPU9250 profili, seviyeleme A/B, simülatör düzeltmeleri), (B) case metninin madde madde teslimatla karşılaştırması, (C) GitHub Pages kontrolü.

---

## A. v0.5.5 doğrulaması

### A.1 Ne değişti (5 commit, `0eb2a51`…`bc01c5b`)

| Değişiklik | Durum |
|---|---|
| Simülatör: sesli callout metni ŝ'den 50 m'ye yuvarlanmış ("In 350 m"), banner ile tutarlı, yanında gerçek mesafe; early/late etiketi işarete göre; canlı durum metni sıfırlanıyor; `roundRect` fallback | Doğrulandı: S4 seed 0'da banner 340 m, metin "In 350 m", gerçek 350 m, "3.5 s early"; low-end sekmesi "2.5 s early"; konsol hatası yok |
| IMU sınıfı tespiti (`detect_imu_class`, `imu_class="auto"`) | Kod okundu, test var, aşağıda |
| `FILTER_PROFILES["mpu9250"]` | Kod okundu, 30 tohum sonuçları JSON ile birebir |
| Seviyeleme sanal ölçümü (`level`, varsayılan kapalı), σ sweep'i, yalnız duruşlarda varyantı | Kod okundu, Jacobian testi var, sweep okundu |
| REPORT §4.6 yeniden yazıldı, §4.7 "Levelling does not help", §5 low-end için sonraki adımlar | Okundu |
| v0.5.5 snapshot: metrik değişmedi, test 178 → 180 | 178 geçti, 2 slow atlandı (2:50) |

### A.2 IMU sınıfı tespiti

- `σ̂ = std(Δω)/√2`, eksen başına, medyan; fark alma gerçek dönüşü siler. Pencere: ilk GNSS fix'inden önceki 10 s (koşu daha geç başlıyorsa ilk pencere).
- Eşik 0.027 rad/s/örnek (100 Hz): Note10 ≤ 0.013 (gürültülü titreşimle bile), MPU9250 ≥ 0.059; geometrik ortalama. 200 low-end koşusunun hepsi ve kontrol edilen tüm Note10 koşuları doğru sınıflandı; Note10 sonuçları bit-birebir aynı (S1, S3, S4).
- Batch'te çoğunluk kararı (bir batch = bir telefon modeli): doğru.
- Raporun uyarısı yerinde: gerçek telefonda OS jiroyu alçak geçirebilir; sensör adı daha güvenli anahtar. Gerçek uygulamada ikisi birlikte (ad varsa ad, yoksa gürültü).

### A.3 MPU9250 profili ve sonuçlar

Ayar kuralları Note10'dan türetilmiş ve koda yazılmış: `gyro_noise` √(ARW² + titreşim²) = 6.3e-3, `gyro_bias_rw` Note10 × BI oranı (30.7), `accel_bias_rw` × 1.08, `p0_bg` 0.4°, `p0_ba` 0.45. ×0.5/×2 sweep'i bir şey değiştirmiyor: "tek parametre çözmez" iddiası ölçülmüş.

| MPU9250 + profil, 30 tohum (`reports/0.5.5/lowend_profile_30.json`) | S1 | S2 | S5 | S4 | 5 km |
|---|---|---|---|---|---|
| ayrım / portal rms | 45 / 37 m | 14 / 16 m | 78 / 85 m | 740 / 1080 m | 206 / 614 m |
| (e/σ)² portal | 4.8 | 2.0 | 34 | 700 | 146 |
| callout k = 0 rms; k = 1 > 1 s geç | 1.5 s; 17 % | 0.5 s; 0 % | 3.4 s; 57 % | 163 s; 90 % | 8.5 s; 93 % |

Rapor tablosu JSON ile birebir. Benim 6 tohumluk sayılarımla uyumlu (S1 47, S2 10, S5 53). §4.6'nın yeni çerçevesi doğru: kısa/düz tünelde onlarca metre (harita yaw'ı tutuyor), dur-kalk ve uzun tünelde kırılıyor, her yerde aşırı güvenli; "sürücü görünümü ±40 m gösterirken araç 1 km ötede, k·σ callout artık korumuyor" cümlesi bu senaryonun en dürüst özeti. Low-end'de uygulamanın σ'ya güvenmemesi (marjı genişlet ya da CV + harita) doğru ürün kararı.

### A.4 Seviyeleme A/B (`reports/0.5.5/level_sigma_sweep_10.json`, 10 tohum)

| Note10, ayrım / portal / (e/σ)² | kapalı | σ 0.01 | σ 0.02 | σ 0.05 |
|---|---|---|---|---|
| S1 | 11.7 / 4.4 / 0.1 | 7.9 / 24.8 / 3.7 | 8.7 / 14.2 / 1.1 | 10.5 / 5.9 / 0.2 |
| S3 | 11.8 / 5.8 / 0.1 | 30 / 164 / 98 | 22 / 12.7 / 0.7 | 8.5 / 8.7 / 0.3 |
| S4 (callout k0) | 28.8 / 6.6 / 0.2 (1.48 s) | 15.0 / 12.7 / 0.7 (0.74) | 14.4 / 8.0 / 0.3 (0.75) | 19.9 / 5.9 / 0.1 (1.02) |
| S5 | 22.5 / 8.4 / 0.3 | 105 / 123 / 38 | 22 / 82 / 13.5 | 19.8 / 7.6 / 0.3 |
| S6 | 12.9 / 5.8 / 0.1 | 30 / 37 / 7.8 | 30 / 31 / 4.9 | 16.4 / 11.2 / 0.6 |
| 5 km | 18.0 / 13.3 / 0.2 | 13.5 / 49.5 / 4.7 | 13.7 / 31 / 1.6 | 15.4 / 15.9 / 0.4 |

Yalnız duruşlarda (30 tohum): hiç etkisi yok, ZUPT zaten tutuyor.

Okumalar:
- Kapalı bırakma kararı doğru.
- Sweep'te gördüğüm ve rapora bir cümle olarak eklenebilecek örüntü: σ 0.01–0.02'de **ayrım iyileşirken portal kötüleşiyor** (S1 4.4 → 14, 5 km 13 → 31) ve (e/σ)² yükseliyor. Mekanizma: seviyeleme P_ss'i küçültüyor, kavis eşleştirmesi küçük σ_s ile daha az düzeltiyor. Yani zarar yalnızca sürücü hız sapması ve eğim değişiminden değil, aşırı güvenin kavis kazancını yemesinden de geliyor.
- σ = 0.05 satırı dengeli okuma: S4 callout 1.48 → 1.02 s, S3/S5 ayrım iyileşiyor, S6 ve 5 km portal ~3 m kötüleşiyor, tutarlılık bozulmuyor. "Harm small, gain small" doğru. İleride denenebilecek tek aday: σ 0.05 ile yalnızca düşük hızda (crawl); şimdi değil.
- Kod: `rate = |ω_y,z|·v` kapısı hem dönüşü hem eğim değişimini eliyor (benim prototipimden iyi); 1 s ortalama rate kullanımı doğru (örnek başına maksimum jiro gürültüsü olurdu, WIP commit'te düzeltilmiş).

### A.5 Küçük notlar

- CHANGELOG'daki "Review 4 (simulator) fixes" ifadesi review 11'i kastediyor; numara yanlış.
- §4.6 tablosunda ilk satır 10, diğerleri 30 tohum; etiketli, yeterli.
- Tag'ler hâlâ yok (v0.5.0–v0.5.5).

---

## B. Case metniyle karşılaştırma

Case metni (Sr. R&D Engineer Positioning, "Through the Tunnel") madde madde:

| Case'in istediği | Durum | Not |
|---|---|---|
| 1. Yaklaşım(lar) önerme | ✓ | ESKF + katmanlar; CV referansı; denenip reddedilen alternatifler (absolute hold, increment, gradyan durumu, seviyeleme) ölçümleriyle |
| 2. Görsel simülatör: tünelli yol, bir veya daha fazla araç, telefonun kaydedeceği sensör verisi, gerçek konum yanında kestirim | ✓ | Tek araç (karar notunda gerekçeli); sensör paneli, sürücü görünümü, canlı mod |
| 3. Simülatörle değerlendirme: ne kadar iyi, nerede kırılıyor, neden | ✓ güçlü | 50 tohum, 8 senaryo + 3 uzunluk, negatif kontroller, §4'te yedi kırılma noktası mekanizmalarıyla |
| Tünel tek şerit, 500 m – 5 km | ✓ | 0.5 / 1 / 3 / 5 km |
| Seçimler (tünel, senaryo, sensör, gerçekçilik) yazılı | Kısmen | DESIGN.md'de tam, **REPORT'ta yok** (aşağıda) |
| Belirsiz yerlerde makul varsayım, yazılı | Kısmen | [ASSUMED] etiketleri var, **konsolide liste REPORT'tan düşmüş** |
| Kod + simülatörü çalıştıran README | ✓ | `requirements.txt`, Python ≥ 3.10, üç komut |
| Write-up: approach | ✓ | §1–2 |
| Write-up: **assumptions** | **✗** | v0.5.3 kısaltmasında eski §7 tablosu düşmüş; "see profiles.py and DESIGN.md §2" ile dışarı gönderiliyor |
| Write-up: **what you simulated and how** | **✗** | Truth modeli, sensör modelleri, GNSS degradasyonu/coasting/multipath, 200 s yaklaşma, senaryo tablosu, Monte Carlo: raporda bir paragraf bile yok; senaryo tablosu yalnız README'de |
| Write-up: results | ✓ | §3 |
| Write-up: limitations | ✓ | §5 |
| Write-up: next with real devices | ✓ | §5 |
| AI usage: nasıl kullanıldı, nerede düzeltildi | ✓ | §6, somut örneklerle; review dosyaları kanıt |
| Opsiyonel ekran kaydı | ✓ eski | mp4 21 Ekim 05:43: harita katmanı sonrası ama k = 1, sürücü görünümü ve sesli metin düzeltmesi öncesi |
| "Anything else you can reasonably assume… research, use and justify" | Kısmen | Harita kullanımı gerekçeli; **reddedilen sensörler** (manyetometre, ışık sensörü, kamera, BLE, OBD, tekerlek frekansı) DESIGN'da var, REPORT'ta geçmiyor |
| Canlı görüşme: "new scenarios and parameter changes live" | Parametre ✓, **senaryo ✗** | serve.py parametreleri çözüyor; yeni yol geometrisi/eğim/duruş `scenario.py` düzenlemesi gerektiriyor |

### B.1 Düzeltmeye değer üç nokta

1. **REPORT'a "Assumptions" tablosu geri konmalı** (~10 satır): montaj rijit ve biliniyor; IMU Note10 Allan [MEAS], 3-GM şekli [ASSUMED]; GNSS 1 Hz, 3 m, korelasyonlu hız gürültüsü, portal öncesi 4 s degradasyon, çıkış multipath; baro 25 Hz, GM ofset, kapalı kabin; harita exact polyline + yanal ofset σ 2 m, uzunluk σ %0.5; araç sideslip 0.4°/(m/s²); sürücü GM hız sapması; callout 300 m, k = 1; çıkış kolu 20°/150 m; **kullanılmayan sensörler ve gerekçesi** (manyetometre: tünel yapısı ve araç gövdesi; ışık sensörü: fikstür aralığı bilinmiyor; kamera/BLE/OBD: ortalama kullanıcıda yok).
2. **§2 ile §3 arasına "What is simulated" yarım sayfa:** truth → sensörler → kestirimci ayrımı (test zorluyor), senaryo tablosu (S1–S6L + uzunluklar), sensör hata terimlerinin tek tek kapatılabildiği, GNSS'in "fades out" davranışı, 200 s yaklaşma manevraları, her sayının 50 tohum olduğu. Case'in "which sensors to simulate and how realistically; these choices are part of what we look at" cümlesinin cevabı bugün DESIGN.md'de saklı.
3. **Canlı senaryo:** canlı şeride dört kontrol (tünel uzunluğu, sabit eğim %, kavis yarıçapı, bir duruş süresi); `_straight_road` ve `SpeedEvent` ile ~30 satır. Yapılmayacaksa `scenario.py`'de yeni senaryo eklemeyi bir kez prova edin; görüşmede kod editörü sahnesi riskli.

Küçükler: mp4'ü son sürümle yeniden kaydetmek (`record_frames.js` hazır, 5 dk); limitasyonlara "one car; a second car would only be another noise realisation" yarım cümlesi.

---

## C. GitHub Pages kontrolü

Buradan doğrulanamadı: ortamın proxy'si `github.io` alanına bağlantıya izin vermiyor (CONNECT 403), Pages API proxy'de kapalı, repo API erişimi için gereken yetki verilmedi. Dolaylı bulgular:

- Remote'ta yalnızca `main` dalı var; `gh-pages` dalı ve `.github/workflows` yok. Pages ancak "Deploy from a branch: `main`, folder `/docs`" ayarıyla açık olabilir; bu ayar dışarıdan görünmez.
- `docs/sim/index.html` commit'li, tek dosya, yalnızca göreli bağlantı; `docs/` altında Jekyll'i rahatsız edecek bir şey yok (alt çizgiyle başlayan dosya yok). Ayar açıksa sayfa olduğu gibi servis edilir; `location.hash` ile sekme seçimi (`…/sim/#S4`) Pages'te de çalışır.

Sizin kontrolünüz (1 dk):
1. Repo → Settings → Pages: Source "Deploy from a branch", Branch `main`, Folder `/docs`; kaydedince "Your site is live at https://alpermutlu2.github.io/Tunnel-navigation/" görünmeli, ilk yayın birkaç dakika.
2. Gizli pencerede `https://alpermutlu2.github.io/Tunnel-navigation/sim/`: sekmeler ve "map in the tunnel" kutusu görünmeli, LIVE şeridi görünmemeli; 3.5 MB, ilk açılış 2–3 s.

Pages açılmayacaksa README'deki adres kaldırılmalı; kırık link teslimatta ilk tıklanacak yer.

---

## D. Açık kalanlar (tüm turlar)

| Madde | Durum |
|---|---|
| v0.5.0–v0.5.5 tag'leri | Push'lanmadı (rapor "tag'ler yazar tarafından konur" diyor) |
| Pages | Doğrulanamadı, yukarıdaki kontrol |
| REPORT: assumptions tablosu, "what is simulated" bölümü, reddedilen sensörler | Açık (B.1) |
| Canlı senaryo kontrolleri | Açık (B.1) |
| mp4 yeniden kayıt | Açık |
| CHANGELOG "Review 4" → "Review 11" | Açık |
| Harita kalitesi 2× hatada aşırı güven | Raporda yazılı, σ'lar gerçek haritada ölçülecek |
