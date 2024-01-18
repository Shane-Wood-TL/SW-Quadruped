# SW-Quadruped
Quadruped Robot similar to the Boston Dynamic's spot


# Version 8
![PXL_20240109_133647437 RAW-01 COVER](https://github.com/Shane-Wood-TL/SW-Quadruped/assets/15756211/72773da9-5f1b-4323-a050-5a22c0420b21)
![PXL_20240109_133659338 RAW-01 COVER](https://github.com/Shane-Wood-TL/SW-Quadruped/assets/15756211/70bbd255-8f86-404e-873b-f104a194f9be)
![PXL_20240109_133707881 RAW-01 COVER](https://github.com/Shane-Wood-TL/SW-Quadruped/assets/15756211/20310985-2406-4ddd-9bba-937ca05e3a2d)


Uses 12x DS32XX motors controlled by a set of PCA9685s
Mircocontroller - ESP32-S3
Gyro: BNO055

Power:
20V in from Dewalt battery (20V, 4A),
20V into buck converter (20V - 6.4V),
6.4V Powers servos,
20V into buck converter (20V - 5V),
5V powers ESP32-S3 as well as everything else
I2C is based on 3V3 logic
