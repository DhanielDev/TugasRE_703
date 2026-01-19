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


# 🤖 Algoritma Navigasi (FSM)

Node navigasi menggunakan 5 state utama:
| State      | Fungsi                                  |
| ---------- | --------------------------------------- |
| **IDLE**   | Menunggu & memilih waypoint             |
| **ROTATE** | Memutar robot menghadap target          |
| **MOVE**   | Maju + belok halus (smooth path)        |
| **NEXT**   | Pindah ke waypoint berikutnya           |
| **DONE**   | Berhenti setelah semua waypoint selesai |

---

## 🚀 Langkah Menjalankan Simulasi
### 🛠️ 1) Build Workspace
Buka terminal:

*cd ~/workspace*
*colcon build*

Setelah selesai:
*source install/setup.bash*

## 🖥️ 2) Jalankan Gazebo + Robot
Terminal 1:

*cd ~/workspace*
*source install/setup.bash*
*ros2 launch articubot_one launch_sim.launch.py*

## 🤖 3) Jalankan Navigasi Autonomous (FSM)

Buka tab terminal baru:

*cd ~/workspace*
*source install/setup.bash*
*ros2 run fsm_nav fsm_nav*


Jika berhasil, akan muncul log seperti:

FSM Waypoint Navigator Started
Going to waypoint 1
Going to waypoint 2
...

📡 Topik ROS yang Dipakai
Topik	Kegunaan

| **/diff_cont/odom**   | Membaca posisi & orientasi robot |
| **/cmd_vel**          | Mengirim perintah gerak ke robot |
	
	







