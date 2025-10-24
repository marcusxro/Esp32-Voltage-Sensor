This project is an ESP32-based smart power monitoring and protection system that measures voltage, current, power, and temperature, and automatically trips the MCB using servo motors when overvoltage is detected. It also logs data to Firebase Realtime Database and sends SMS alerts via Air780E GSM Module.

**Components Used**
Component	Description
ESP32	Main microcontroller for processing and Wi-Fi connection
ZMPT101B Voltage Sensor	Measures AC voltage
ACS712 Current Sensor (30A)	Measures AC current
DS18B20 Temperature Sensor	Measures temperature
Air780E GSM Module	Sends SMS alerts
16x2 I2C LCD Display	Displays real-time readings
Servo Motor x2	Controls MCB tripping and reset mechanism
Buzzer	Audible alarm on overvoltage
Buck Step-Down Converter (5V)	Powers servo motors and sensors
Resistors, jumper wires, breadboard	General circuit connections

 **Wiring Connections**
ESP32 Pin Mapping
Module / Device	Pin on ESP32	Notes
ZMPT101B Voltage Sensor	34	Analog input
ACS712 Current Sensor	32	Analog input
DS18B20 Temperature Sensor	26	OneWire data pin (add 4.7kΩ pull-up resistor to 3.3V)
Air780E GSM (RX)	4	Connects to TX of GSM module
Air780E GSM (TX)	2	Connects to RX of GSM module
Servo Motor 1	12	MCB switch servo 1
Servo Motor 2	13	MCB switch servo 2
Buzzer	5	Active buzzer output
I2C LCD SDA	21	I2C data line
I2C LCD SCL	22	I2C clock line
5V Power Supply	VIN / External 5V	Ensure enough current for servos
