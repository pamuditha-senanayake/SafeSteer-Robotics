# 🚡 SafeSteer – AI-Powered Smart Anti-Drowsiness Steering Wheel

**SafeSteer** is an intelligent, real-time driver monitoring and alert system built using AI, computer vision, and IoT.
It detects drowsiness and yawning using a Pi Camera, monitors biometric data from sensors, and triggers escalating alerts to ensure driver safety.

---

## 🚗 Features

### 🔍 Drowsiness Detection (Level 1)

* Uses **OpenCV** and facial landmarks (EAR/MAR) to detect:

  * Eye closure
  * Yawning
* Triggers **audio alert** using `paplay` if drowsiness is detected

### ❤️ Sensor Integration (via Arduino)

* Reads:

  * **Heart rate**
  * **Touch sensor**
  * **Accelerometer**
* Sends JSON data to Raspberry Pi via serial
* Level 1 is triggered when **camera + any sensor** indicate drowsiness

### 🧠 Emergency Alert System (Level 3)

* If Level 2 (sensor-only) escalation is detected by Arduino:

  * Pi triggers **WhatsApp alert** via **Twilio API**
  * Sends emergency message to a pre-set contact

### ☁️ ThingSpeak Dashboard

* Uploads live sensor data + camera metrics:

  * EAR (Eye Aspect Ratio)
  * MAR (Mouth Aspect Ratio)
  * Frame counters
* Real-time visualization via Chart.js frontend

---

## 🧰 Technologies Used

* **Raspberry Pi 3B**
* **Arduino Uno**
* **Python 3.x** (multithreaded)
* **OpenCV** for computer vision
* **Twilio API** (WhatsApp messaging)
* **ThingSpeak** (IoT data monitoring)
* **FFmpeg**, **paplay**, **serial**, **requests**, **json**

---

## 🗈 System Architecture

```
[Pi Camera]         [Arduino Sensors]
     |                     |
     v                     v
[Python Script with Threads] <--- Serial JSON
     |
     +---> [OpenCV Detection] ---> Level 1
     +---> [Sensor Analysis]  ---> Level 1 / Level 3
     +---> [ThingSpeak API]
     +---> [Audio Alert / Twilio]
```

---

## 🛠️ Setup Instructions

### 1️⃣ Prerequisites

* Raspberry Pi OS installed
* Arduino connected to Pi via USB
* Camera connected (accessed via `/dev/video17`)
* Dependencies installed:

```bash
sudo apt install ffmpeg pulseaudio
pip install opencv-python requests pyserial
```

---

### 2️⃣ Start Arduino Sensor Data

* Upload the Arduino sketch from `arduino/level2_monitor.ino`
* Sensors: touch, pulse sensor, accelerometer (connected as per diagram)

---

### 3️⃣ Run the Python Script on Pi

```bash
python3 safesteer_main.py
```

✅ This will:

* Read serial data from Arduino
* Start camera stream using `cv2.VideoCapture('/dev/video17', cv2.CAP_V4L2)`
* Detect drowsiness and trigger alerts
* Upload values to ThingSpeak every few seconds

---

## 🧪 Data Fields (ThingSpeak)

| Field | Description              |
| ----- | ------------------------ |
| 1     | Eye Aspect Ratio (EAR)   |
| 2     | Mouth Aspect Ratio (MAR) |
| 3     | Frame Counter (EAR)      |
| 4     | Frame Counter (MAR)      |
| 5     | Heart Rate               |
| 6     | Touch Sensor Status      |
| 7     | Accelerometer Status     |

---

## 📂 Repo Structure

```
SafeSteer/
├── safesteer_main.py
├── arduino/
│   └── level2_monitor.ino
├── utils/
│   ├── camera_utils.py
│   ├── sensor_handler.py
│   └── twilio_alert.py
├── media/
│   └── alert.wav
├── README.md
└── requirements.txt
```

---

## 🧠 AI Logic (Facial Landmarks)

* Eye Aspect Ratio (EAR) is computed from eye landmarks to detect prolonged closure
* Mouth Aspect Ratio (MAR) is used to detect yawning
* If thresholds are crossed for 30+ frames, Level 1 alert is triggered

---

## 📞 Twilio WhatsApp Alert

```python
from twilio.rest import Client

client = Client(ACCOUNT_SID, AUTH_TOKEN)
message = client.messages.create(
    from_='whatsapp:+14155238886',
    to='whatsapp:+94XXXXXXXXX',
    body='🚨 SafeSteer Alert: Drowsiness detected. Immediate attention required!'
)
```

---

## 🌐 Demo & UI Dashboard

* React + Chart.js dashboard displays:

  * Live EAR/MAR graphs
  * Sensor status
  * Timestamped alert log
* Can be viewed via local IP or deployed on Firebase/Netlify

---

## 🧪 Testing Modes

* You can simulate sensor inputs using `simulated_serial_input.py`
* Useful for demos without physical sensors

---

## 👨‍💻 Author

**Pamuditha Senanayake**
LinkedIn: [@pamuditha-senanayake-87794357](https://www.linkedin.com/in/pamuditha-senanayake-87794357/)
Email: [pamudithasenanayake@gmail.com](mailto:pamudithasenanayake@gmail.com)

---

## 📜 License

MIT – feel free to use, modify, and contribute!

---
