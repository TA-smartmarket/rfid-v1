# Uji logika firmware

Jalankan dari root proyek dengan Python dan g++ tersedia di PATH:

```text
python test/host/run.py
```

Runner mengompilasi `src/main.cpp` yang sebenarnya dengan pengganti API Arduino,
HX711, MFRC522, SPI, dan Preferences. Setiap skenario berjalan di proses baru.
Semua file hasil kompilasi dibuat di direktori sementara dan dibersihkan otomatis.
Output JSON diperiksa agar tetap valid serta berlabel simulasi dan belum terkirim.

Skenario mencakup startup, kalibrasi negatif 174 g, faktor tersimpan, RFID offline,
UID 10 byte, barang masuk/keluar, batas 20 g, scan bertumpuk/duplikat/timeout,
sensor putus/pulih, perintah RX, input invalid/panjang, tampilan kg, noise,
saturasi ADC, kegagalan penyimpanan, dan waktu yang melewati batas uint32.
Indikator diuji hanya menyala pada ITEM_ADDED, mati setelah 300 ms termasuk saat
millis berputar, dan tetap melayani perintah serial saat menyala. Mock LEDC
memeriksa urutan setup timer sebelum attach/write dan kegagalan setup; suara
serta frekuensi output fisik belum diuji oleh tes host.

Build target nyata: `pio run`. Tes host tidak menguji timing listrik, SPI fisik,
flash NVS nyata, mounting load cell, atau akurasi berat. Urutan pengujian hardware
dan wiring ada di `CODEX.md`.
