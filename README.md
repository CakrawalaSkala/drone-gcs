# 🚁 **CAKSA GCS (Ground Control Station)**

**CAKSA GCS** adalah aplikasi Ground Control Station berbasis web modern untuk memonitor dan mengendalikan drone menggunakan protokol MAVLink (ArduPilot/PX4).

Aplikasi ini menggunakan arsitektur **Decoupled**:

- **Frontend:** React + Leaflet  
- **Backend:** Python + MAVSDK  

---

## ✨ **Fitur Utama**

### 📡 **Real-time Telemetry**
Pantau data penerbangan secara langsung:
- Altitude  
- Speed  
- Heading  
- Battery  
- GPS Position  

### 🗺 **Live Map & Dual View**
- Pelacakan posisi drone berbasis Leaflet (Dark Mode)  
- **Auto-Pan** mengikuti drone  
- **Dual View:** Map ↔ Camera mock view  

### 🛩 **Artificial Horizon (HUD)**
Menampilkan Pitch & Roll secara real-time.

### 🎯 **Mission Planner (Autonomous)**
- **Figure 8 Autonomous Mission** (Lemniscate)  
- **Auto-Land** setelah misi selesai  
- **Upload Mission**
- **Start Mission**

### ✅ **Pre-Flight Check**
Sistem inspeksi sebelum terbang:
- IMU (Gyro/Accel)
- Magnetometer
- GPS 3D Fix & Satellites
- Battery health

### 🎮 **Keyboard Offboard Control**
Kendalikan drone secara manual:
- **WASD:** Gerak horizontal  
- **Space:** Naik  
- **Arrow Keys:** Yaw & altitude  

---

## 🛠 **Teknologi yang Digunakan**

### **Frontend**
- React + TypeScript (Vite)
- Tailwind CSS
- Leaflet + React-Leaflet
- Lucide React

### **Backend**
- Python 3.9+
- MAVSDK
- WebSockets
- Asyncio

---

## ⚙ **Instalasi**

### 1️⃣ Setup Backend (Python)

Masuk ke folder backend, lalu install dependensi:

```bash
pip install mavsdk websockets asyncio
```

Jalankan backend:

```bash
python gcs_backend.py
```

Jika sukses, muncul:

```
WebSocket listening on ws://localhost:8080
*** DRONE CONNECTED! ***
```

---

## 2️⃣ Setup Frontend (React)

Masuk ke folder frontend:

```bash
npm install
npm run dev
```

Buka **http://localhost:5173**

---

## 🛫 **Menjalankan Simulator (SITL)**

### **Opsi A — Windows (Mission Planner)**

##### 1. Install **Mission Planner**
##### 2. Buka tab **SIMULATION**
##### 3. Pilih drone **Multirotor**
##### 4. Tekan **Ctrl + F**
##### 5. Klik **Mavlink**
##### 6. Pilih **UDP Host**
##### 7. Masukkan port **14550**
####
---

### **Opsi B — Linux/WSL**

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --map --console
```

Tunggu sampai muncul **GPS: 3D Fix**

---

## 🚀 **Workflow Menjalankan Aplikasi**

##### 1. Jalankan **SITL**  
##### 2. Jalankan **Backend**  
   ```bash
   python gcs_backend.py
   ```
##### 3. Jalankan **Frontend**  
   ```bash
   npm run dev
   ```

---

## ✈️ Setelah Connect

### **1. Pre-Flight Check**
- Masuk tab **Pre-Flight Check**
- Tekan **RUN DIAGNOSTICS**

### **2. Takeoff Manual**
- LOITER / HOLD  
- ARM  
- TAKEOFF  

### **3. Mission Planner**
- Klik **UPLOAD**  
- Klik **START**  
  → Drone membentuk angka 8, lalu mendarat otomatis.

### **4. Mode Keyboard (Offboard)**
- Klik **OFFBOARD (MANUAL)**
- Gunakan kontrol keyboard.

---

## 🎮 **Kontrol Keyboard (Offboard Mode)**

| Tombol | Fungsi | Arah |
|--------|--------|-------|
| W | Maju | Forward |
| S | Mundur | Backward |
| A | Geser kiri | Strafe Left |
| D | Geser kanan | Strafe Right |
| Spasi | Naik | Throttle Up |
| ↑ | Naik | Altitude Up |
| ↓ | Turun | Altitude Down |
| ← | Putar kiri | Yaw Left |
| → | Putar kanan | Yaw Right |

---

## 🔌 **Menghubungkan ke Drone Asli**

##### 1. Hubungkan Flight Controller (Pixhawk/Cube)  
##### 2. Sambungkan **SiK Telemetry** ke laptop  
##### 3. Cek COM Port  
##### 4. Edit `gcs_backend.py`:

### Untuk Telemetry Radio
```python
CONNECTION_STRING = "serial://COM3:57600"
```

### Untuk Simulator (default)
```python
CONNECTION_STRING = "udp://:14550"
```

---

## 📝 **Lisensi**

Proyek ini digunakan untuk pengembangan Ground Control Station CAKSA.

