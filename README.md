# rfid-v1

```text
feat: tambahkan indikator barang masuk dan perjelas log serial

- Nyalakan LED GPIO25 dan buzzer GPIO26 selama 300 ms saat ITEM_ADDED
- Perbaiki inisialisasi PWM buzzer untuk mengatasi error LEDC
- Tambahkan pembatas *** pada log TARE, CAL, RFID, dan ITEM_ADDED
- Sinkronkan CODEX.md dengan implementasi terbaru

Validasi: build ESP32 berhasil dan 15 skenario tes host lulus.
```
