# Tunnel-navigation repo incelemesi, 4. tur: simülatör (v0.5.4) ve düşük sınıf IMU

Repo: https://github.com/alpermutlu2/Tunnel-navigation
İncelenen sürüm: `59e52bb` (v0.5.4, 21 Ekim 11:10)
Önceki turlar: v0.5.0 (`df3fdfa`), v0.5.1 (`4c4626b`), v0.5.2 (`7abf54e`), v0.5.3 (`42a08bf`)
İnceleme: 21 Ekim 2026, 11:20–11:45

Bu belge iki tespiti kapsar: (A) v0.5.4 simülatörünün uçtan uca kontrolü, (B) düşük sınıf IMU için ölçülmüş seçenekler.

---

## A. Simülatör kontrolü (v0.5.4)

### A.1 Kapsam ve yöntem

- `docs/sim/index.html` dosya olarak (Python'suz) headless Chromium'da sürüldü: 10 sekme, harita anahtarı, k ve mesafe kaydırıcıları, klavye, çift tıkla tam ekran, sürücü görünümü.
- `tools/serve.py` ile canlı mod: `/api/ping`, `/api/run`, `/controls`; seed/IMU/harita değişiklikleriyle yeniden koşular.
- Sayfaya gömülü `ALL` verisi ayrıştırılıp harita açık/kapalı varyantları sayısal olarak karşılaştırıldı.
- ESKF testleri HEAD'de koşuldu: 12/12.
- v0.5.4 rapor ve CHANGELOG farkları okundu: k = 1 gerekçesi, k = 0 / k = 1 ayrı satırlar, "right moment" tanımı, snapshot farkının yalnızca callout metriklerinden geldiği açıklaması. Hepsi yerinde.

### A.2 Doğrulananlar

| Kontrol | Sonuç |
|---|---|
| Dosya olarak açılış | 1.8 s, konsol hatası yok, 3.5 MB |
| Sekmeler | 8 senaryo + 2 kırılma (30 Pa/km, low-end); kırılma sekmelerinde harita anahtarı devre dışı |
| Harita açık/kapalı | Gömülü iki varyant gerçekten farklı: 5 km, t = 341 s yanal 36.4 → 1.5 m; S4 varyantları yanal 25.6 vs 3.7 m; haritasız görünümde tahmin aracı tünelin yanında |
| k kaydırıcısı | Tarayıcıda yeniden hesaplıyor (S1 seed 0: k = 1 −1.5 s, k = 0 −1.0 s) |
| Klavye | ← → 1 s, Shift 10 s, Space play/pause doğru |
| Tam ekran | Çift tık headless'ta bile açtı; playback bar panele taşınıyor |
| Canlı mod | LIVE şeridi açılıyor; S4 seed 3 12.9 s, low-end 9.5 s; harita kapatma yeniden koşuyor; zaman konumu korunuyor; `/controls` 200 |
| Sürücü görünümü | Heading-up, yola oturtulmuş konum, "No GPS · estimated ±2σ", hız. Low-end sekmesinde "±46 m" gösterirken gerçek hata 1.5 km: filtrenin aşırı güveni uygulama ekranında da görünüyor, mülakatta iyi bir an |

### A.3 Düzeltilmesi iyi olacaklar (küçük)

1. **Sesli callout metni ile banner tutarsız.** Baloncuk sabit "In 300 m, take Exit A"; banner aynı anda ŝ'ye göre mesafe. S4 seed 0, k = 1: banner 343 m, metin 300 m, gerçek 350 m. Metin ŝ'den 50 m'ye yuvarlanmış mesafe söylemeli ("In 350 m") ya da "In about 300 m". Birincisi hem doğru hem uygulamaların yaptığı şey.
2. `callStat` etiketi sabit "(− early)"; low-end sekmesinde "99.5 s (− early)" yazıyor, oysa geç. İşarete göre early/late.
3. Canlı modda ayarlar sırayla varsayılana dönünce durum metni "running…" kalıyor (süper edilen istek sessizce atılıyor). `liveDefault()` dalında `liveStat` temizlensin. Süper edilen istek sunucuda yine de koşup kilidi ~10 s tutuyor; kabul edilebilir.
4. README'deki GitHub Pages adresi (`alpermutlu2.github.io/Tunnel-navigation/sim/`) buradan doğrulanamadı; Pages'in `/docs` klasörüne açık olduğu push sonrası bir kez tarayıcıda kontrol edilmeli.
5. 3.5 MB sayfa: Pages için sorun değil, mobilde birkaç saniye. Gerekirse sensör izleri 1 Hz.
6. `roundRect` Firefox 112+ / Safari 16+; üç satır fallback isteğe bağlı.

Bu haliyle simülatör case metnindeki her maddeyi karşılıyor: yol + tünel, gerçek ve tahmini araç, sensör verisi, canlı parametre değişikliği; artı sürücünün gerçekten göreceği ekran.

---

## B. Düşük sınıf IMU: ne yapılabilir (ölçülmüş)

### B.1 Sorunun yeri

Suvorkin Tablo 5 (MPU9250): accel BI 0.8–1.3e-3 m/s² (Note10 ile aynı sınıf), jiro BI 3–7e-3 rad/s = 0.2–0.4 °/s, ARW ~20 °/√h. Sorun neredeyse tamamen jiro. Tasarımın jiroya bağımlı tek yeri pitch tutuşu; yaw artık harita katmanından geliyor. Jiro 1 s'de ~0.3° pitch hatası → g·sin 0.3° ≈ 0.05 m/s² → 180 s'de yüzlerce metre: raporun km'leri bu.

Kaynak notu: Tablo 5'teki jiro BI 900–1400 °/h, MPU9250 datasheet tipik değerinin ~100 katı ve bir dev-board ölçümü. Gerçek bir düşük sınıf telefonun jirosu büyük ihtimalle bundan iyidir; raporda "low-end profili kötümser olabilir, gerçek telefonda ölçülmeli" cümlesi yerinde olur.

### B.2 Seçenek 1: profil ayarı + harita katmanı birlikte (repoda var, raporda yok)

§4.6'daki "1.3 km" harita katmanından önceki tek retune'dan geliyor. Bugünkü kodla (harita açık, v0.5.4) ve jiro Q/P0 MPU9250'ye göre ayarlanınca (`gyro_noise` 6.5e-3, `gyro_bias_rw` 7e-3/√180, `p0_bg` 0.5°, `accel_bias_rw` 1.3e-3/√180, `p0_ba` 0.35), 6 tohum:

| MPU9250 | S1 | S2 | S5 | S4 | 5 km |
|---|---|---|---|---|---|
| varsayılan ayar (raporun durumu), portal | 1.7 km | 440 m | 1.5 km | 1.7 km | |
| profil ayarı + harita, ayrım / portal | 53 / **47 m** | 8 / **10 m** | 73 / **53 m** | 690 / 990 m | 210 / 530 m |
| (e/σ)² portal | 7.9 | 0.8 | 12.4 | 420 | 85 |
| callout k = 0 rms | 1.8 s | 0.2 s | 3.1 s | 64 s | 8.6 s |

Okumalar:
- Düz ve kısa tünellerde low-end "kapsam dışı" değil, onlarca metre. Harita yaw'ı tuttuğu için jironun yarısı devre dışı kalıyor.
- Tutarlılık hâlâ kötü (8–12): ayar tek seferlik değil sistematik olmalı; `gyro_noise` ARW'den, `gyro_bias_rw` BI ve τ'dan (τ = 5 s bileşeni için σ·√(2/τ) ≈ 3e-3 rad/s/√s), P0 profile göre. `gyro_bias_rw` = 3e-3 denemesi S1'de 47 → 45, S5'te 53 → 68 (kötü): tek parametreyle çözülmüyor, profil tablosu gerekir.
- S4 ve 5 km yüzlerce metre: jironun hızlı bias sapması ile pitch güncellemeleri arasında kaybedilen zaman; mevcut yapıda kapanmıyor.
- Rapor §4.6 bu tabloyla "kapsam dışı"ndan "kısa/düz tünelde çalışır, uzun ve dur-kalkta çalışmaz"a çevrilmeli. Daha doğru ve daha iyi bir cevap.

### B.3 Seçenek 2: başlangıçta IMU sınıfı tespiti

Profili seçmek için: park halinde 30 s jiro bant RMS'i (Allan tipi hızlı kontrol) ya da doğrudan Android sensör adı/üreticisi. Ucuz, şart; seçenek 1 bununla anlamlı.

### B.4 Seçenek 3: seviyeleme (gravity-direction) sanal ölçümü, prototip sonuçları

Fikir: cruise'da ve duruşlarda 1 s ortalama özgül kuvvet = −Cᵀg + b_a; ölçüm `z = (−Ĉᵀg + b̂_a) − f̄`, `H = [−Cᵀ[g]× | I]` (δθ ve δb_a), R = σ_a² + VRW. Pitch'i jirodan değil ivmeölçerden alır (1 s'de 1e-4 rad, jirodan 30× iyi).

Bulgular (6 tohum, scratch prototip, repoya dokunulmadı):
- İlk sürüm kavislerde merkezcil ivmeyi tilt sandı ve Note10'u bile bozdu (S1 portal 5 → 166 m, S2 3.5 → 115 m). Dönüş kapısıyla (|ω_z|·v > 0.3 m/s²) düzeldi. NHC yanal satırındaki dersin aynısı.
- Low-end'e S1/S5'te yaramıyor: ölçüm b_f + g·θ lumped terimini görüyor, yeni bir şey ayırmıyor. S4'te 2× (990 → 440 m), ama ancak ivmeölçer tabanlı ayrı bir "düz gidiyor" dedektörüyle; filtrenin kendi cruise dedektörü low-end'de hiç tetiklenmiyor (ivme kestirimi gürültülü, 5 s ortalama eşiği aşıyor).
- **Beklenmedik yan bulgu, Note10:** S4'te ayrımdaki hata 32 → 12 m, callout rms 1.66 → 0.67 s. Duruşlarda tilt + lumped bias sıfırlanıyor, bir tür "tilt-ZUPT". Zayıf senaryoda büyük kazanç.
- Bedeli: R ayarı gerekiyor; (e/σ)² S2'de 1.1, 5 km'de 1.2; S4 portal 7 → 11.5, 5 km portal 8 → 27 m. Varsayılan yapmadan önce 30 tohumluk A/B ve σ_a sweep'i şart.

| Note10 + seviyeleme (dönüş kapılı) | S1 | S2 | S4 | S5 | 5 km |
|---|---|---|---|---|---|
| ayrım, kapalı → açık | 12.7 → 11.0 | 1.7 → 4.9 | **32 → 12.3** | 21.8 → 18.1 | 16.4 → 15.7 |
| portal, kapalı → açık | 5.1 → 6.6 | 3.5 → 8.7 | 7.1 → 11.5 | 4.8 → 4.2 | 8.2 → 27.3 |
| (e/σ)², açık | 0.2 | 1.1 | 0.5 | 0.1 | 1.2 |
| callout k = 0 rms, kapalı → açık | 0.52 → 0.47 | 0.20 → 0.26 | **1.66 → 0.67** | 1.18 → 1.01 | 0.66 → 0.65 |

Öneri: ayrı bir A/B (`EskfConfig.level`, varsayılan kapalı), hedef S4 callout; low-end için değil Note10 için değerli.

### B.5 Seçenek 4: kalan taban için adaylar (ölçülmedi)

- Barometreyi 1 s ortalama yerine 25 Hz ham ve kendi korelasyon modeliyle kullanmak: pitch'i daha sık tutar. Baro grade tabanı 2.5 mm/s² ≈ 100 s'de 12 m; S1'de teorik ~20 m mümkün, bugün 47.
- Jiro bias'ını random walk değil hızlı bileşenli GM (τ = 5/50/500 s) olarak modellemek: filtre geri dönüşü bekler, Q şişirmekten daha doğru.
- DESIGN 5.1'deki CV + harita geri düşüşü: dur-kalk ve uzun tünelde low-end için INS yerine.
- Android'in kalibre jirosu (`TYPE_GYROSCOPE`) vs ham: OS'nin duruşta bias kalibrasyonu turn-on'u alır, tünel içi sapmayı almaz.

### B.6 Öncelik

1. Seçenek 1 ve 2 bugün yapılabilir: profil tablosu (Note10 / MPU9250) + `--override imu_lowend` ile 30 tohum; §4.6 yeniden yazılır. Beklenen: S1/S2/S5 onlarca metre, S4 ve 5 km "burada kırılıyor, sebebi jironun hızlı bias sapması".
2. Seçenek 3 ayrı A/B, Note10 S4 için.
3. Seçenek 4 sonraki iş; rapora "next steps".

---

## C. Açık kalanlar (önceki turlardan)

- v0.5.0–v0.5.4 tag'leri push'lanmamış; rapor "tag'ler yazar tarafından konur" diyor.
- Harita kalitesi: 2× hatada filtre hâlâ aşırı güvenli ((e/σ)² 1.7–2.1); raporda yazılı, σ'lar gerçek haritada ölçülmeli.
- Callout: k = 1 gerekçesi yazıldı; sesli metin mesafesi (A.3 madde 1) ile tutarlılık kaldı.
