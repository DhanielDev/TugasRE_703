# 🤖 FSM Waypoint Navigation — Gazebo ROS 2 Simulation  

<h1 align="center">
  Proyek Simulasi Navigasi Robot di Gazebo  
</h1>

<h3 align="center">
  Finite State Machine (FSM) + Waypoint Navigation berbasis Odometry
</h3>

<p align="center">
  🎓 Proyek Robotika & Simulasi oleh <b>Dhaniel Beny Wardhana</b> <br>
  🤖 Robot differential drive disimulasikan di Gazebo <br>
  🧠 Navigasi otonom berbasis FSM (IDLE → ROTATE → MOVE → NEXT → DONE) <br>
  📍 Menggunakan waypoint manual dan feedback odometry
</p>

---

## 🧠 Deskripsi Singkat

Proyek ini bertujuan untuk mengembangkan **sistem navigasi otonom berbasis Finite State Machine (FSM)** pada robot differential drive di lingkungan simulasi **Gazebo (ROS 2 Humble).**

Robot akan:
- Mengikuti daftar **waypoint** yang sudah ditentukan  
- Memutar badan terlebih dahulu (**ROTATE**)  
- Bergerak maju sambil melakukan koreksi arah (**MOVE — smooth turning**)  
- Berpindah ke waypoint berikutnya (**NEXT**)  
- Berhenti otomatis saat selesai (**DONE**)  

Kontrol gerak dikirim melalui topik **`/cmd_vel`**, sedangkan posisi robot dibaca dari **`/diff_cont/odom`**.

---

## 📁 Struktur Workspace

workspace/
│
├── src/
│ ├── articubot_one/
│ │ ├── launch/
│ │ │ └── launch_sim.launch.py
│ │ │
│ ├── worlds/
│ │ └── obstacles.world
│ │
│ └── fsm_nav/
│ ├── setup.py
│ ├── package.xml
│ └── fsm_nav/
│ └── fsm_nav.py 
│
└── build/
└── install/
└── log/

---

## 🌍 Dunia Gazebo

File dunia simulasi ditempatkan di:

workspace/src/articubot_one/worlds/obstacles.world


Pastikan di **launch file** terdapat baris seperti:

```python
world_file = os.path.join(
    get_package_share_directory('articubot_one'),
    'worlds',
    'obstacles.world'
)

---

## 🤖 Algoritma Navigasi (FSM)

Node navigasi menggunakan 5 state utama:
| State      | Fungsi                                  |
| ---------- | --------------------------------------- |
| **IDLE**   | Menunggu & memilih waypoint             |
| **ROTATE** | Memutar robot menghadap target          |
| **MOVE**   | Maju + belok halus (smooth path)        |
| **NEXT**   | Pindah ke waypoint berikutnya           |
| **DONE**   | Berhenti setelah semua waypoint selesai |

---

---

## 🧠 Deskripsi Singkat

Proyek ini bertujuan untuk mengembangkan **sistem navigasi otonom berbasis Finite State Machine (FSM)** pada robot differential drive di lingkungan simulasi **Gazebo (ROS 2 Humble).**

Robot akan:
- Mengikuti daftar **waypoint** yang sudah ditentukan  
- Memutar badan terlebih dahulu (**ROTATE**)  
- Bergerak maju sambil melakukan koreksi arah (**MOVE — smooth turning**)  
- Berpindah ke waypoint berikutnya (**NEXT**)  
- Berhenti otomatis saat selesai (**DONE**)  

Kontrol gerak dikirim melalui topik **`/cmd_vel`**, sedangkan posisi robot dibaca dari **`/diff_cont/odom`**.

---
