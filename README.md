# IoT-Based Voice Controlled and Obstacle Detection System for Visually Impaired Wheelchair Users

## Project Overview
This project aims to develop an **IoT-based system** that integrates **voice control**, **obstacle detection**, and **fall detection** capabilities to provide visually impaired wheelchair users with a safe and independent mobility experience. The system uses **ESP32 microcontrollers**, **ultrasonic sensors**, and **MPU6050** for real-time monitoring and control, with **Google Voice Assistant** and **Blynk Cloud** for voice commands and data visualization.

## Key Features
- **Voice Control**: Users can navigate the wheelchair using voice commands via Google Voice Assistant.
- **Obstacle Detection**: Ultrasonic sensors detect obstacles in the wheelchair's path and trigger an emergency stop.
- **Fall Detection**: The MPU6050 sensor detects falls and sends distress notifications to caregivers.
- **Safety Alerts**: A buzzer alerts users of obstacles, and notifications are sent to caregivers in case of emergencies.
- **Real-Time Monitoring**: Data is visualized on the Blynk Cloud platform for real-time monitoring.

## Hardware Components
- **ESP32 Microcontroller**: For processing and connectivity.
- **HC-SR04 Ultrasonic Sensor**: For obstacle detection.
- **MPU6050**: For fall detection (accelerometer and gyroscope).
- **L298N Motor Driver**: For controlling DC motors.
- **DC Motors**: For wheelchair movement.
- **Piezo Buzzer**: For auditory alerts.
- **3.7V Lithium-Ion Batteries**: For power supply.

## Software Tools
- **Arduino IDE**: For programming the ESP32.
- **Blynk Cloud**: For IoT integration and data visualization.
- **Google Voice Assistant**: For voice command recognition.
- **IFTTT**: For triggering actions based on voice commands.
- **GitHub**: For version control and code management.

## Documentation
The project includes comprehensive documentation covering:
- **System Requirements**: Functional and non-functional requirements.
- **Design Diagrams**: Use case, activity, sequence, and context diagrams.
- **Testing Protocols**: Black box testing results for each module.
- **User Manuals**: Instructions for setting up and using the system.
- **Code Documentation**: Detailed comments and explanations in the code.

## Setup Instructions
1. **Hardware Setup**:
   - Connect the ESP32, ultrasonic sensor, MPU6050, motor driver, and DC motors as per the circuit diagram.
   - Power the system using the 3.7V lithium-ion batteries.

2. **Software Setup**:
   - Install the Arduino IDE and necessary libraries (`WiFi.h`, `BlynkSimpleEsp32.h`, `Adafruit_MPU6050.h`, etc.).
   - Upload the provided code to the ESP32.
   - Set up the Blynk Cloud platform and link it to the ESP32.
   - Configure Google Voice Assistant and IFTTT for voice command integration.

3. **Testing**:
   - Test the system by issuing voice commands and placing obstacles in the wheelchair's path.
   - Verify fall detection and safety alerts.

## Results
- The system successfully executed voice commands with a response time of under **2 seconds**.
- Obstacle detection responded within **1 second**, and fall detection notifications were sent within **3 seconds**.
- The prototype demonstrated **95% accuracy** in voice recognition and **98% effectiveness** in obstacle avoidance.

## Future Enhancements
- Integrate **GPS navigation** for outdoor use.
- Implement **machine learning** for path planning and obstacle classification.
- Scale up the prototype for real-world testing with actual wheelchair users.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Special thanks to **Dr. Joseph Orero** and **Madam Julliet Kirui** for their guidance and supervision.
- Gratitude to the **Makerspace Lab** and team members for their support during the project.

---

For more details, refer to the [project documentation](/docs) or contact [olivemenorah@gmail.com](mailto:olivemenorah@gmail.com).
