# 🚗 Collision Avoidance System using CAN Protocol  

## **🔹 Overview**  
The **Collision Avoidance System using CAN Protocol** is designed to enhance vehicular safety by detecting obstacles and preventing potential collisions. The system utilizes **ultrasonic and IR sensors** to monitor the surroundings and detect obstacles in real-time. Additionally, a **temperature sensor** is integrated to track the vehicle's temperature, ensuring stable operation and preventing overheating. 

Communication between system components is established using the **Controller Area Network (CAN) protocol**, employing **MCP2551 transceivers** to enable efficient, low-latency data exchange between two **STM32F407** microcontrollers. The system provides immediate feedback through an **LCD display**, alerting the driver when an obstacle is detected. 

---

## **🛠️ System Components**  
- **Microcontrollers:** STM32F407 (x2)  
- **Communication Module:** MCP2551 CAN Transceivers  
- **Sensors:** Ultrasonic and IR Sensors for obstacle detection  
- **Temperature Sensor:** Monitors vehicle temperature  
- **LCD Display:** Provides real-time obstacle alerts  
- **Power Supply:** Ensures stable operation of all components  

---

## **⚙️ System Working**  
1. **Obstacle Detection:** Ultrasonic and IR sensors continuously scan the vehicle's surroundings. If an obstacle is detected, the sensor sends data to the **STM32F407** microcontroller.  
2. **Data Processing:** The microcontroller processes the sensor data and determines the proximity of the detected obstacle.  
3. **CAN Communication:** If necessary, a warning message is transmitted over the **CAN bus** using **MCP2551 transceivers** to alert the second microcontroller.  
4. **Driver Notification:** The processed information is displayed on an **LCD screen**, warning the driver of nearby obstacles.  
5. **Temperature Monitoring:** The system continuously monitors the vehicle’s temperature, and if overheating is detected, an alert is generated.  

---

## **📈 Key Features & Benefits**  
- **Real-Time Obstacle Detection:** Ensures the vehicle detects obstacles quickly and accurately.  
- **Reliable Communication:** CAN protocol provides efficient and stable communication between system components.  
- **Enhanced Safety:** Immediate driver alerts help in preventing collisions.  
- **Temperature Monitoring:** Protects the vehicle from overheating and potential damage.  
- **Low Latency Response:** Ensures quick system reaction for effective collision avoidance.  

---

## **📷 Images**  
*Figure 1: STM32F407g Board*  
<img src="https://techtonics.in/wp-content/uploads/2024/03/tech1464-1.jpg" alt="System Architecture" width="300">  

*Figure 2: Ultrasonic sensor* 
<img src="https://images.theengineeringprojects.com/image/webp/2018/10/Introduction-to-HC-SR04.jpg.webp?ssl=1" alt="Sensor Setup" width="300">  

*Figure 3: IR Sensor* 
<img src="https://pijaeducation.com/wp-content/uploads/2019/09/Arduino-IR-Collision-Detection-Module-Pin-Outs.jpg" alt="Sensor Setup" width="400">  

*Figure 4: Temperature Sensor*  
<img src="https://circuitdigest.com/sites/default/files/inlineimages/u4/LM35-Sensor-Pinout.jpg" alt="Sensor Setup" width="400">  




---

## **📌 Conclusion**  
The **Collision Avoidance System using CAN Protocol** is a reliable and efficient solution for enhancing vehicle safety. By leveraging **real-time sensors, CAN communication, and LCD feedback**, the system ensures timely obstacle detection and driver alerts. With its robust architecture, it contributes to reducing accidents and improving overall vehicular safety.  

---
