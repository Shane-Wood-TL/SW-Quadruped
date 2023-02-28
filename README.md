# TL-Quadruped
Quadruped Robot based upon similar to that of the boston dynamic's spot
Inspriation for the current design is take from the Spot Mirco Project as well as some of James Bruton's designs

Uses 12x MG996R motors controlled by a PCA9685
The mircocontroller it is designed around is a ESP32-S3
Gyro: BNO055

Current Changes: New PCB


The Design has went through 5 major revisions, starting with the newest is 
# Version 5
![20230208_222907](https://user-images.githubusercontent.com/15756211/221731168-f6a6e609-83d0-4232-abb2-ad7591a1b5c8.jpg)


# Version 4
![20220828_210802](https://user-images.githubusercontent.com/15756211/221730226-4fbdf62b-3c94-46ec-b05f-481931281b5a.jpg)

## Issues
Weak overall body
the way motors are connected to pieces leaves a lot of play
wires being consumed by the design
3d printed bearings not working well for hip joints

# Version 3
![20220517_001230](https://user-images.githubusercontent.com/15756211/221730566-71d9c5f6-14c3-4a44-a9af-61d5357af755.jpg)

## Issues
awful way for the motor's outputs to be connected

# Version 2
![20220221_223218](https://user-images.githubusercontent.com/15756211/221730659-7e29562d-d5d2-4009-89f7-66db6911ea2f.jpg)

## Issues
Too big for the given motors
Lots of flex in the body
Too large and unstable to do anything

# Version 1
![20211109_224533](https://user-images.githubusercontent.com/15756211/221730854-8c0e765d-71b7-4a99-bce5-fe729e7d66ee.jpg)

## Issues
Mirco Controller struggling to perform enough trig in a quick manner (could have been a motor issue)
Way too weak of motors (sg90s)
similar issues in stabilty and body strength as the version that follows


