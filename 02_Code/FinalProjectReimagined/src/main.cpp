#include "variables.cpp"


void setup() {
  
  Serial.begin(9600);
  Wire.begin(17,15); //SDA, SCL
  SPI.begin(18, 8, 10);

  Serial.println("SimpleRx Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  Serial.println("radio Alive");

  Serial.println("alive");
  Serial.println("IC2 alive");

  #ifdef Ugyro
    bno.begin();
    //set up gyro
    Serial.println("gyro started");
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    //bno.setExtCrystalUse(true);
    delay(1000); 
    Serial.println("gyroalive");
    yPID.SetOutputLimits(-45.0,45.0);
    zPID.SetOutputLimits(-45.0,45.0);

        //set the pid mode
    yPID.SetMode(AUTOMATIC);
    zPID.SetMode(AUTOMATIC);
    Serial.println("PID alive");
  #endif
  

  //Set up PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  

  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //where in the walk cycle the robot is at. Way to share state across all movements
  aLeg.setCycle(0);
  bLeg.setCycle(3);
  cLeg.setCycle(3);
  dLeg.setCycle(0);

  mainKinematics(testHeightW, 0, 0, aHip,0,0,0);
  delay(200);
  mainKinematics(testHeightW, 0, 0, cHip,0,0,0);
  mainKinematics(testHeightW, 0, 0, bHip,0,0,0);
  delay(200);
  mainKinematics(testHeightW, 0, 0, dHip,0,0,0);

  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
}

void loop() {
  getData();
  showData();


  
  #ifdef Ugyro  
    bno.getEvent(&event);
    delay(5);
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;
   yPID.Compute();
    zPID.Compute();
  #endif
  #ifndef Ugyro
    yRot = 0;
    zRot = 0;
  #endif

if(payload.eStop != 1){
  wakeup_9();
  if(oldState !=payload.state){
    aLeg.setCycle(0);
    bLeg.setCycle(3);
    cLeg.setCycle(3);
    dLeg.setCycle(0);
  }
  switch (payload.state) {
    case 0:{ //standing
      standing_0();
      break;
    }
    case 1:{ //IK mode
      IK_1();
      break;
    }
    case 2:{//FWalk
      FWalk_2();
      break;
    }
    case 3:{ //
      FTurn_3();
      break;
    }
    case 4:{ //user
      User_4();
      break;
    } 
    case 5:{ //used to install new motors
      all_90s();
    }
    default:
      break;
    }
  }else{
    Default_9(); //turns off motors
  }
  oldState = payload.state;
}