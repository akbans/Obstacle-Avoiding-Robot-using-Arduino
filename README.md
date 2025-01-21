# Obstacle Avoiding 2WD Robot

This repository contains the code and circuit diagram for an obstacle-avoiding 2-wheel drive (2WD) robot using an Arduino microcontroller, an L298N motor driver, an ultrasonic sensor, and an sg90 servo motor. The robot detects obstacles, evaluates the best path, and maneuvers accordingly to avoid collisions.

---

## Features
- **Obstacle Detection**: Uses an ultrasonic sensor for obstacle detection and distance measurement.
- **Servo Scanning**: Provides 180° obstacle coverage using a servo motor.
- **Obstacle Count**: Keeps track of the number of obstacles encountered during operation.
- **Speed Control**: Implements PWM for dynamic motor speed adjustment.
- **Debugging Logs**: Real-time monitoring via Serial Monitor for debugging and tracking.

---

## Components Used
- **Arduino Uno** (or compatible board)
- **L298N Motor Driver**
- **Ultrasonic Sensor (HC-SR04)**
- **Servo Motor**
- **2 DC Motors** (connected to the wheels)
- **Chassis and Wheels**
- **12V Power Source** (e.g., battery pack)
- **Jumper Wires**
- **Breadboard** (optional)

---

## Files in This Repository
1. **`code.ino`**: The Arduino source code for the robot.
2. **`circuit.jpg`**: The circuit diagram for wiring the components.

---

## Getting Started

### Prerequisites
- Install the following Arduino libraries:
  - [Servo](https://www.arduino.cc/en/Reference/Servo) (Built-in)
  - [NewPing](https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home)

### Steps to Build the Robot
1. **Assemble the Hardware**:
   - Follow the circuit diagram in `circuit.jpg` to connect the components correctly.
   - -Ensure the GND of motor driver and arduino are connected together.
2. **Load the Code**:
   - Open `code.ino` in the Arduino IDE.
   - Connect the Arduino board to your computer.
   - Compile and upload the code to the Arduino.
3. **Test the Robot**:
   - Place the robot in an open space.
   - Power the robot and observe its obstacle-avoiding behavior.

---

## How It Works
1. **Obstacle Detection**:
   - The ultrasonic sensor measures the distance to nearby obstacles.
2. **Servo Scanning**:
   - The servo motor rotates to scan the surroundings for better pathfinding.
3. **Decision-Making**:
   - The robot decides the best direction based on sensor data.
4. **Movement**:
   - The L298N motor driver controls the wheels for forward, backward, left, or right movement.

---

## Customization
- **Motor Speed**: Adjust the `motorSpeed` variable in the code.
- **Obstacle Detection Range**: Modify the `MAX_DISTANCE` or logic thresholds.
- **Debugging**: Use the Serial Monitor to view real-time logs for fine-tuning.

---

## Future Improvements
- Integrate 2 IR sensors for more accurate detection.
- Add Bluetooth or Wi-Fi control for remote operation.
- Implement a maze-solving feature.

---

## Acknowledgments
- Arduino Community for excellent support and resources.
- Libraries: Servo and NewPing for simplifying hardware control.
