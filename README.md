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

Diasumsikan nama workspace kamu:

