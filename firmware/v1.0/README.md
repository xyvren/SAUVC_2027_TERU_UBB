# AUV SAUVAC — Firmware Release v1.0 Autonomous
**Sistem Kendali Robot Selam Tak Berawak (RP2040 Dual-Core) — Kualifikasi SAUVC 2027**  
*Jurusan Teknik Elektro, Fakultas Sains dan Teknik, Universitas Bangka Belitung (UBB)*

---

## 📌 Ringkasan Berkas Rilis Biner

Direktori ini menyediakan berkas biner resmi hasil kompilasi **Firmware v1.0 Autonomous** yang siap langsung dipasang (*flashing*) ke mikrokontroler Raspberry Pi Pico W tanpa memerlukan instalasi toolchain compiler PlatformIO di komputer Anda.

| Nama Berkas | Format | Ukuran | Metode Penggunaan Utama | SHA-256 Checksum |
| :--- | :---: | :---: | :--- | :--- |
| **`firmware_v1.0.uf2`** | UF2 (*USB Flashing Format*) | **1.069.056 byte** (~1.0 MB) | **Metode 1:** Drag-and-drop mass storage saat mode BOOTSEL USB (Board baru/kosong) | `b677d97e72261f7400f35ea7d3bf5224e3ab1116bdbd5f551e4c34e8fd8b2c12` |
| **`firmware_v1.0.bin`** | Raw Binary (*LittleFS OTA*) | **534.380 byte** (~522 KB) | **Metode 2:** Pembaruan nirkabel Over-The-Air via Web Terminal (Tanpa buka hull) | `e22efe32c6412f0d915ac70d245fdb91d40de8b3c0f6ec97379515a2541b5e85` |

---

## 🚀 Panduan 3 Metode Flashing Firmware

### Metode 1: Flashing Awal via USB (BOOTSEL Drag-and-Drop)
*Gunakan metode ini jika board Raspberry Pi Pico W masih baru, memori flash kosong, atau sedang dalam tahap pemulihan total (recovery).*

1. Pastikan baterai utama AUV dimatikan atau kabel daya dilepas.
2. Tekan dan tahan tombol fisik **BOOTSEL** (tombol putih kecil pada papan Pico W).
3. Sambil terus menahan tombol BOOTSEL, tancapkan kabel micro-USB dari Pico W ke laptop/komputer. Lepaskan tombol BOOTSEL setelah kabel tertancap.
4. Komputer Anda akan mengenali Pico W sebagai media penyimpanan USB bernama **`RPI-RP2`**.
5. Seret (*drag-and-drop*) atau salin berkas **[`firmware_v1.0.uf2`](./firmware_v1.0.uf2)** langsung ke dalam drive **`RPI-RP2`**.
6. Drive `RPI-RP2` akan otomatis tertutup (*eject* sendiri). Mikrokontroler me-reboot secara otomatis.
7. Dalam waktu ~2 detik, LED onboard Pico W akan berkedip perlahan menandakan hotspot Wi-Fi SoftAP **`AUV_UBB_JUARA`** telah aktif.

---

### Metode 2: Flashing Nirkabel via OTA (Metode Utama di Lapangan / Poolside)
*Gunakan metode ini saat pengujian di tepi kolam renang tanpa perlu membongkar tabung akrilik atau kompartemen kedap air (hull), sehingga keawetan seal O-ring dan ketahanan IP68 tetap terjaga sempurna.*

1. Nyalakan daya AUV SAUVAC.
2. Sambungkan koneksi Wi-Fi laptop, smartphone, atau tablet ke jaringan hotspot:
   - **SSID Hotspot:** `AUV_UBB_JUARA`
   - **Kata Sandi (WPA2):** `ubbjuara2027`
3. Buka peramban web (*Google Chrome / Mozilla Firefox / Safari*) dan akses alamat:
   ```text
   http://192.168.4.1
   ```
4. Pada antarmuka Web Terminal, klik tab **`📦 OTA Firmware Update`**.
5. Klik tombol **Pilih Berkas Firmware** dan pilih berkas **[`firmware_v1.0.bin`](./firmware_v1.0.bin)** dari laptop Anda.
6. Klik tombol **Flash Firmware Nirkabel**.
7. Bilah kemajuan (*progress bar*) akan menampilkan proses pengunggahan dari 0% hingga 100%.
8. Setelah pengunggahan selesai, mikrokontroler akan otomatis memverifikasi biner dan melakukan restart mandiri dalam waktu kurang dari 5 detik. Hubungkan kembali perangkat Anda ke hotspot.

---

### Metode 3: Kompilasi & Flashing Manual via PlatformIO CLI
*Gunakan metode ini jika Anda melakukan perubahan logika pada kode sumber C++ di direktori `src/`.*

1. Pastikan Anda telah menginstal **PlatformIO Core (CLI)** atau ekstensi PlatformIO di VS Code.
2. Buka terminal di direktori utama repositori (`auv-6thruster-picow-platformio`).
3. Jalankan kompilasi kode:
   ```bash
   pio run
   ```
4. Untuk mengunggah langsung ke Pico W via kabel USB CDC Serial (tanpa menekan tombol BOOTSEL):
   ```bash
   pio run --target upload
   ```
5. Berkas biner keluaran otomatis diperbarui di folder `.pio/build/picow/`.

---

## ⚙️ Kredensial & Parameter Baku Sistem (Default)

| Parameter | Nilai Default | Keterangan |
| :--- | :--- | :--- |
| **Wi-Fi SSID** | `AUV_UBB_JUARA` | Nama hotspot SoftAP Pico W |
| **Wi-Fi Password** | `ubbjuara2027` | Keamanan WPA2-PSK |
| **IP Address** | `192.168.4.1` | Gateway Web Console & REST API |
| **Port HTTP** | `80` | Web Terminal & OTA Flasher |
| **Baud Rate Serial** | `115200 bps` | USB CDC Serial Monitor |
| **PWM Neutral** | `1500 µs` | Stop / Diam seluruh 6 Thruster |
| **Target Kedalaman** | `50 cm` | Closed-Loop PID Sensor MS5803 |
| **Failsafe Watchdog**| `600 ms` | Netral otomatis jika komunikasi terputus |

---

## ✅ Prosedur Verifikasi Pasca-Flashing

Setelah proses flashing selesai, jalankan langkah verifikasi cepat berikut:
1. Akses Web Terminal di `http://192.168.4.1` atau buka Serial Monitor (115200 baud).
2. Jalankan perintah diagnostik mandiri:
   ```text
   /selftest
   ```
3. Pastikan seluruh subsistem berstatus **PASS**:
   - `MS5803 Pressure` : **PASS** (Pembacaan mbar dan suhu normal)
   - `MPU-6050 IMU` : **PASS** (Pembacaan pitch dan roll terdeteksi)
   - `Digital Compass` : **PASS** (Heading kompas terdeteksi)
   - `EEPROM Storage` : **PASS** (`v1.0 Struct & Checksum OK`)
   - `LittleFS Flash` : **PASS** (Storage OTA terpasang)
   - `Wi-Fi SoftAP` : **PASS** (`AUV_UBB_JUARA` aktif)
   - `Thruster T1–T6` : **PASS** (Seluruh pulsa motor 0.2s terhubung)
4. Robot siap dioperasikan untuk pengujian misi otonom kualifikasi SAUVC 2027.
