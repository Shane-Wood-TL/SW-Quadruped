# Version 7
Fixed the body issues of v6 with the use of carbon fiber tubes, similar to v6 otherwise.
![PXL_20230803_034527840 PORTRAIT (1)](https://github.com/Townlake101/TL-Quadruped/assets/15756211/6cbb1202-86b7-4fbc-993a-1dcd48be1b01)



# Version 6
![PXL_20230622_041525787](https://github.com/Townlake101/TL-Quadruped/assets/15756211/7380e80b-6586-435f-85e7-c34cd7a7470a)

Similar to V5
The main goal is to reduce tolerance issues within V5 as well as make maintenance easier
Increase mobility range of legs which was reduced in the in/out direction due to the square nature of v5
Switch from a single PCB to 3, 1 main controller board with 2 servo driver boards, which communicate over a custom protocol that consists of I2C and some digital pins
Instead of using wires to connect these boards ethernet wires will be used instead, this is also the main way the controller will have access to other devices, it features 4 ethernet jacks
Given the digital pins that now go to the servo boards, a way to detect if the robot's leg is on the ground is also planned with the help of keyboard switches in the robot's foot


# Version 5
![yeet](https://user-images.githubusercontent.com/15756211/226653216-69dbfabc-1a9a-4a78-b281-26a315f22245.jpg)





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
an awful way for the motor's outputs to be connected

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
similar issues in stability and body strength as the version that follows



