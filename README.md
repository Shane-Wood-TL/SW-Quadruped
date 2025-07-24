# SW-Quadruped
Quadruped Robot similar to the Boston Dynamic's spot


# Version 9
![PXL_20240917_164015504](https://github.com/user-attachments/assets/7b31491c-248b-4416-b33d-0919cec255af)
![PXL_20240917_164736266](https://github.com/user-attachments/assets/57532b39-5ef5-424d-a58b-f9cbf948885a)
![PXL_20240917_164745462](https://github.com/user-attachments/assets/25650e58-d6f5-4fb5-a532-7c94cf0e3633)



Uses 12x DS32XX motors controlled by a set of PCA9685s <br>
<t> Planned upgrade to [these servos](https://github.com/Shane-Wood-TL/CAN_Servo/tree/main)<br>
Mircocontroller - ESP32-S3<br>
Gyro: BNO055 - Over CAN bus<br>
[Controller](https://github.com/Shane-Wood-TL/Universal-Robo-Controller)<br>
Lidar over ESP-NOW <br><br>


Power:
20V in from Dewalt battery (20V, 2A), <br>
20V into buck converter (20V - 6.4V),<br>
6.4V Powers servos, <br>
20V into buck converter (20V - 5V), <br>
5V powers ESP32-S3 as well as everything else <br>
I2C is based on 3V3 logic <br>
