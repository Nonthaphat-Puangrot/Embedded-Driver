# Driver Behavior Monitoring System

A real-time embedded system for monitoring and analyzing driver behavior in racing simulations. This project combines Arduino-based hardware controls with a Flask web application to provide comprehensive telemetry tracking, safety alerts, and parental control features.

## 🚀 Features

### Hardware Controller
- **Steering Control**: Rotary encoder-based steering wheel with full range (-32767 to +32767)
- **Pedal System**: Analog throttle and brake pedals with real-time input processing
- **Gear Shifting**: Manual gear system (Neutral + 5 forward gears)
- **RPM Simulation**: Dynamic RPM output with gear-based limiting
- **LED Matrix Display**: Visual feedback for gear position and system status
- **FreeRTOS Integration**: Multi-threaded task management for responsive controls

### Web Dashboard
- **Real-time Telemetry**: Live visualization of steering, throttle, brake, gear, and RPM data
- **Interactive Charts**: Historical data tracking with Chart.js integration
- **Connection Monitoring**: Real-time MQTT connection status indicators
- **Alert System**: Critical event notifications for unsafe driving patterns
- **Parental Control Panel**: Configurable speed and throttle limitations

### Safety Features
- **Behavior Monitoring**: Detects sustained high-speed/high-throttle driving
- **Alert Notifications**: Visual and audible warnings for critical events
- **Parental Controls**: OTA-configurable limits for maximum gear and throttle
- **Emergency Override**: Quick-access safety controls from the web interface

## 📋 Requirements

### Hardware
- Arduino UNO R4 WiFi
- Rotary encoder (for steering wheel)
- 2x Analog pedals (throttle and brake pedals)
- 2x Push buttons (gear up/down)
- LED matrix (built-in on UNO R4)
- Voltmeter for display RPM signal

### Software Dependencies

#### Arduino Libraries
```
ezButton
WiFi
MQTT
ArduinoGraphics
Arduino_LED_Matrix
ArduinoJson
Arduino_FreeRTOS
```

#### Python Requirements
```
Flask>=2.3.0
Flask-SocketIO>=5.3.0
paho-mqtt>=1.6.1
python-socketio>=5.9.0
```

### Infrastructure
- Mosquitto MQTT Broker (v2.0+)
- Python 3.8 or higher

## 🔧 Installation

### 1. Arduino Setup

1. Install the Arduino IDE (v2.0+)
2. Install required libraries via Library Manager
3. Open `DriverMonitoringController.ino`
4. Configure WiFi credentials in the code:
   ```cpp
   const char* ssid = "your_wifi_ssid";
   const char* password = "your_wifi_password";
   ```
5. Set MQTT broker IP address:
   ```cpp
   const char* mqtt_server = "your_mqtt_broker_ip";
   ```
6. Upload to Arduino UNO R4 WiFi

### 2. Python Environment Setup

```powershell
# Create virtual environment
python -m venv .venv

# Activate virtual environment
.\.venv\Scripts\activate
```

### 3. MQTT Broker Setup

1. Download and install [Mosquitto](https://mosquitto.org/download/)
2. Configure `mosquitto.conf` with appropriate settings
3. Start the broker:
   ```powershell
   mosquitto.exe -c mosquitto.conf -v
   ```

## 🚀 Running the System

### Start the MQTT Broker
```powershell
mosquitto.exe -c mosquitto.conf -v
```

### Start the Web Application
```powershell
.\.venv\Scripts\python.exe app.py
```

### Access the Dashboard
Open your browser and navigate to:
```
http://localhost:5000
```

Or access from other devices on the network:
```
http://<your-ip-address>:5000
```

## 📡 MQTT Topics

### Published by Arduino
- `car/driver1/steering` - Steering wheel position (-32767 to 32767)
- `car/driver1/throttle` - Throttle percentage (0-100)
- `car/driver1/brake` - Brake percentage (0-100)
- `car/driver1/gear` - Current gear (0-5)
- `car/driver1/rpm` - RPM value (PWM 0-140)
- `car/driver1/alerts` - Safety alerts (JSON format)

### Subscribed by Arduino
- `car/driver1/parental_control_settings` - OTA configuration updates (JSON)

## 🎮 Usage

### Basic Operation
1. Power on the Arduino controller
2. Start the MQTT broker and web application
3. Open the dashboard in your browser
4. Use the physical controls (steering wheel, pedals, gear shifters)
5. Monitor telemetry in real-time on the dashboard

### Parental Control Configuration
1. Navigate to the Parental Control section in the dashboard
2. Toggle the enable switch
3. Set maximum allowed gear (1-5)
4. Set maximum throttle percentage (0-100)
5. Changes are applied instantly via OTA updates

### Alert Handling
- Alerts appear automatically when unsafe driving is detected
- Click "Dismiss" to acknowledge the alert
- Click "Engage Parental Control" to immediately activate safety limits

## 🏗️ Project Structure

```
Embedded_final/
├── DriverMonitoringController.ino   # Arduino firmware
├── app.py                            # Flask web application
├── templates/
│   └── index.html                    # Web dashboard interface
├── mosquitto.conf                    # MQTT broker configuration
├── readme.md                         # This file
└── .venv/                           # Python virtual environment
```

## 🔒 Safety Features

### Alert Triggers
- Sustained high gear (≥4) with high throttle (≥70%) for 10+ seconds
- Configurable thresholds in the Arduino code or website

### Parental Control Limits
- **Maximum Gear**: Restricts gear selection (e.g., limit to gear 3)
- **Maximum Throttle**: Caps throttle input percentage
- **Real-time Enforcement**: Applied immediately without restart

## 🛠️ Troubleshooting

### Arduino not connecting to MQTT
- Verify WiFi credentials
- Check MQTT broker IP address
- Ensure broker is running and accessible
- Check firewall settings

### Web dashboard not receiving data
- Verify MQTT broker is running
- Check console for connection errors
- Ensure Arduino is publishing data (check Serial Monitor)
- Verify MQTT topics match in Arduino and Python code

### Encoder/pedal inputs not responding
- Check wiring connections
- Verify pin assignments in code
- Test with Serial Monitor output
- Check for loose connections

## 📝 Configuration

### Arduino Pin Configuration
```cpp
CLK_PIN = 2        // Rotary encoder CLK
DT_PIN = 3         // Rotary encoder DT
SW_PIN = 4         // Rotary encoder button
GEAR_UP_PIN = 6    // Gear up button
GEAR_DOWN_PIN = 7  // Gear down button
rpmOutPin = 9      // PWM output for RPM
forwardPedalPin = A1  // Throttle pedal analog input
brakePedalPin = A2    // Brake pedal analog input
```

### Web Application Settings
Edit `app.py` to configure:
- MQTT broker address: `MQTT_BROKER = "10.23.166.129"`
- Server port: `port=5000`
- Default parental control values

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Commit your changes with clear messages
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is provided as-is for educational purposes.

## 👥 Authors

Embedded Systems Project Team

## 🙏 Acknowledgments

- Arduino community for libraries and support
- Flask and Socket.IO documentation
- Mosquitto MQTT broker project

---

**Last Updated**: November 26, 2025
