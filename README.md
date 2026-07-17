# ShuttleBot + ESPServo

Panduan singkat penggunaan dua ESP32:

- `shutlebot` = robot utama, Bluetooth, roda, pelontar, sudut.
- `SERVO` = kontrol servo arm, gripper, dan conveyor.
- Komunikasi antar ESP32 memakai UART `Serial2`.

## 1. Upload Program

Upload masing-masing folder PlatformIO:

```bash
cd SERVO
~/.platformio/penv/bin/pio run --target upload

cd ../shutlebot
~/.platformio/penv/bin/pio run --target upload
```

Jika lewat VS Code PlatformIO, buka folder masing-masing lalu klik **Upload**.

## 2. Wiring UART Antar ESP32

Hubungkan silang TX/RX:

| ShuttleBot | ESPServo | Fungsi |
| --- | --- | --- |
| GPIO0 TX | GPIO16 RX2 | kirim command ke servo |
| GPIO34 RX | GPIO17 TX2 | terima status dari servo |
| GND | GND | ground bersama |

Catatan:

- Semua GND wajib tersambung: ESP32 ShuttleBot, ESP32 Servo, driver motor, supply.
- GPIO0 adalah pin boot ESP32. Jika sulit upload/boot, lepas kabel GPIO0 sementara atau pindahkan TX ShuttleBot ke pin output kosong lain lalu ubah kode.
- Jangan sambungkan langsung sumber motor/servo ke 5V ESP32. Pakai supply terpisah sesuai arus motor/servo.

## 3. Bluetooth

1. Nyalakan ShuttleBot.
2. Pair dari HP/laptop ke Bluetooth bernama `OmniBot-ESP32`.
3. Buka aplikasi Bluetooth Serial.
4. Kirim command huruf sesuai tabel.

## 4. Command Servo + Conveyor

Command ini dikirim dari Bluetooth ke ShuttleBot, lalu ShuttleBot meneruskan ke ESPServo.

| Command | Fungsi |
| --- | --- |
| `Y` | Conveyor hidup terus |
| `W` | Servo ambil jalan 1 kali |
| `N` | Conveyor mati, servo kembali netral |
| `T` | Arm naik manual |
| `O` | Gripper tutup manual |
| `?` | Cek status ESPServo |

Cara pakai utama:

1. Kirim `Y` untuk menyalakan conveyor terus.
2. Kirim `W` saat ingin servo ambil 1 kali.
3. Kirim `N` untuk mematikan conveyor dan reset servo.

## 5. Command Gerak ShuttleBot

| Command | Fungsi |
| --- | --- |
| `F` | maju |
| `B` | mundur |
| `L` | geser kiri |
| `R` | geser kanan |
| `G` | diagonal maju-kiri |
| `I` | diagonal maju-kanan |
| `H` | diagonal mundur-kiri |
| `J` | diagonal mundur-kanan |
| `Q` | putar kiri |
| `E` | putar kanan |
| `S` | stop roda |
| `0`..`9` | atur speed roda |
| `+` | tambah speed roda |
| `-` | kurangi speed roda |

## 6. Command Pelontar

| Command | Fungsi |
| --- | --- |
| `K` | pelontar ON |
| `M` | pelontar OFF |
| `P0`..`P9` | level speed pelontar |
| `]` | tambah speed pelontar |
| `[` | kurangi speed pelontar |

## 7. Command Sudut Pelontar

| Command | Fungsi |
| --- | --- |
| `Z` | homing sudut ke limit bawah |
| `A0`..`A43` | set sudut absolut |
| `U` | sudut naik 1 derajat |
| `D` | sudut turun 1 derajat |
| `C` | stop gerakan sudut |
| `V` | tampilkan status sudut |

Contoh:

- `A30` = sudut pelontar ke 30 derajat.
- `A43` = sudut pelontar ke 43 derajat.

## 8. Emergency Stop

| Command | Fungsi |
| --- | --- |
| `X` | stop roda, pelontar, sudut, lalu kirim `N` ke ESPServo |

Gunakan `X` jika robot tidak terkendali.

## 9. Troubleshooting

### Servo tidak bergerak

- Cek supply servo cukup kuat.
- Cek GND servo supply tersambung ke GND ESP32.
- Cek kabel ShuttleBot TX ke ESPServo RX2.
- Kirim `?` dari Bluetooth, lihat apakah ada balasan `ESPServo OK`.

### Conveyor tidak hidup

- Kirim `Y`.
- Cek wiring L298N: `ENA GPIO25`, `IN1 GPIO32`, `IN2 GPIO33` di ESPServo.
- Cek supply motor conveyor.

### ESP32 sulit upload

- Lepas kabel dari `GPIO0` ShuttleBot saat upload.
- Setelah upload selesai, pasang lagi.

### Bluetooth tidak muncul

- Pastikan board memakai ESP32 klasik/WROOM, bukan ESP32-S3/C3 tanpa Bluetooth Classic SPP.
- Nama Bluetooth: `OmniBot-ESP32`.
