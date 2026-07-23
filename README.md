# 🤖 ShuttleBot + ESPServo

Sistem ini menggunakan **dua buah ESP32** yang saling berkomunikasi melalui **UART (Serial2)**.

## 📦 Arsitektur Sistem

| Board                  | Fungsi                                                                             |
| ---------------------- | ---------------------------------------------------------------------------------- |
| **ShuttleBot (ESP32)** | Kontrol robot utama, Bluetooth, roda omni, pelontar, dan pengaturan sudut pelontar |
| **ESPServo (ESP32)**   | Kontrol lengan servo, gripper, dan conveyor                                        |

Komunikasi antar board menggunakan **UART Serial2**.

---

# 🚀 Upload Program

Upload firmware untuk masing-masing board.

### Menggunakan PlatformIO CLI

```bash
cd SERVO
~/.platformio/penv/bin/pio run --target upload

cd ../shutlebot
~/.platformio/penv/bin/pio run --target upload
```

### Menggunakan VS Code

1. Buka folder **SERVO**.
2. Klik **Upload**.
3. Setelah selesai, buka folder **shutlebot**.
4. Klik **Upload**.

---

# 🔌 Wiring UART Antar ESP32

Hubungkan kedua ESP32 secara silang.

| ShuttleBot  | ESPServo     | Fungsi                        |
| ----------- | ------------ | ----------------------------- |
| GPIO0 (TX)  | GPIO16 (RX2) | Mengirim command ke ESPServo  |
| GPIO34 (RX) | GPIO17 (TX2) | Menerima status dari ESPServo |
| GND         | GND          | Ground bersama                |

> **Penting**
>
> * Semua **GND** harus tersambung menjadi satu (ESP32, driver motor, servo, dan power supply).
> * GPIO0 merupakan pin boot ESP32. Jika upload gagal, lepaskan kabel pada GPIO0 sementara atau gunakan pin TX lain dan sesuaikan program.
> * Jangan memberi daya motor atau servo dari pin **5V ESP32**. Gunakan power supply terpisah dengan arus yang memadai.

---

# 📱 Koneksi Bluetooth

1. Nyalakan ShuttleBot.
2. Pair perangkat dengan Bluetooth bernama:

```
OmniBot-ESP32
```

3. Buka aplikasi **Bluetooth Serial Terminal**.
4. Kirim command sesuai tabel berikut.

---

# 🎮 Command Servo & Conveyor

Command dikirim melalui Bluetooth ke ShuttleBot, kemudian diteruskan ke ESPServo.

| Command | Fungsi                                          |
| ------- | ----------------------------------------------- |
| **Y**   | Conveyor ON (berjalan terus)                    |
| **W**   | Servo melakukan satu siklus pengambilan         |
| **N**   | Conveyor OFF dan servo kembali ke posisi netral |
| **T**   | Arm naik manual                                 |
| **O**   | Gripper menutup manual                          |
| **?**   | Meminta status ESPServo                         |

### Alur Operasi

```text
Y  → Conveyor berjalan
↓
W  → Servo mengambil objek satu kali
↓
N  → Conveyor berhenti dan servo kembali ke posisi awal
```

---

# 🚗 Command Gerak ShuttleBot

| Command | Fungsi                |
| ------- | --------------------- |
| F       | Maju                  |
| B       | Mundur                |
| L       | Geser kiri            |
| R       | Geser kanan           |
| G       | Diagonal maju-kiri    |
| I       | Diagonal maju-kanan   |
| H       | Diagonal mundur-kiri  |
| J       | Diagonal mundur-kanan |
| Q       | Putar kiri            |
| E       | Putar kanan           |
| S       | Stop                  |
| 0–9     | Atur kecepatan roda   |
| +       | Tambah kecepatan      |
| -       | Kurangi kecepatan     |

---

# 🎯 Command Pelontar

| Command | Fungsi                        |
| ------- | ----------------------------- |
| K       | Pelontar ON                   |
| M       | Pelontar OFF                  |
| P0–P9   | Atur level kecepatan pelontar |
| ]       | Tambah kecepatan              |
| [       | Kurangi kecepatan             |

---

# 📐 Command Sudut Pelontar

| Command | Fungsi                 |
| ------- | ---------------------- |
| Z       | Homing ke limit bawah  |
| A0–A43  | Atur sudut absolut     |
| U       | Naik 1°                |
| D       | Turun 1°               |
| C       | Hentikan gerakan sudut |
| V       | Tampilkan status sudut |

### Contoh

```text
A30
```

Menggerakkan pelontar ke **30°**.

```text
A43
```

Menggerakkan pelontar ke **43°**.

---

# 🛑 Emergency Stop

| Command | Fungsi                                                                                     |
| ------- | ------------------------------------------------------------------------------------------ |
| **X**   | Menghentikan seluruh sistem (roda, pelontar, sudut) dan mengirim command **N** ke ESPServo |

Gunakan command ini ketika robot mengalami kondisi tidak terkendali.

---

# 🔍 Troubleshooting

## Servo Tidak Bergerak

✅ Pastikan supply servo mencukupi.

✅ Pastikan ground servo dan ESP32 tersambung.

✅ Periksa koneksi:

```
ShuttleBot TX
        │
        ▼
ESPServo RX2
```

✅ Kirim command:

```
?
```

Jika komunikasi normal, ESPServo akan membalas:

```
ESPServo OK
```

---

## Conveyor Tidak Berjalan

Pastikan command berikut sudah dikirim:

```
Y
```

Periksa wiring L298N:

| Pin | GPIO   |
| --- | ------ |
| ENA | GPIO25 |
| IN1 | GPIO32 |
| IN2 | GPIO33 |

Pastikan supply motor conveyor mencukupi.

---

## ESP32 Sulit Di-upload

* Lepaskan kabel pada **GPIO0** ShuttleBot.
* Upload ulang firmware.
* Pasang kembali kabel setelah proses upload selesai.

---

## Bluetooth Tidak Muncul

Pastikan menggunakan **ESP32 Classic / WROOM** yang mendukung Bluetooth Classic (SPP).

Bluetooth akan muncul dengan nama:

```
OmniBot-ESP32
```

---

# 📋 Ringkasan Command

## ShuttleBot

| Fitur     | Command           |
| --------- | ----------------- |
| Gerak     | F B L R G H I J S |
| Speed     | 0–9 + -           |
| Pelontar  | K M P0–P9 [ ]     |
| Sudut     | Z A0–A43 U D C V  |
| Emergency | X                 |

## ESPServo

| Fitur       | Command |
| ----------- | ------- |
| Conveyor ON | Y       |
| Servo Ambil | W       |
| Reset Servo | N       |
| Arm Naik    | T       |
| Gripper     | O       |
| Status      | ?       |

---

## 🏁 Alur Pengoperasian Singkat

```text
Upload Firmware
        │
        ▼
Hubungkan UART + GND
        │
        ▼
Nyalakan Robot
        │
        ▼
Pair Bluetooth "OmniBot-ESP32"
        │
        ▼
Kirim Command
        │
        ├── Gerak Robot
        ├── Pelontar
        ├── Sudut
        └── Servo & Conveyor
```
