#include <Wire.h>
#include <Servo.h>

Servo servo;
int pos = 0;

int mpu = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;
int16_t time = 0;

bool AcxFirstLoop, AcXFirstCrossover = true;
int AcXLastTime, AcXCurrentTime, AcXAverage;
int AcXUpperThreshold, AcXLowerThreshold, AcXAverageThreshold;

bool GyZWithinTrough = false;
int GyZDownTime, timeBtwPks=600;
int timeBtwPksThreshold, GyzThreshold; 

bool AcXDirection;
int AcXCycleCount = 0;

int AcXInitialAcceleration;
int AcXCycleLengths[2];

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  servo.attach(13);

  pinMode(11, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(mpu);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Set Thresholds
  AcXUpperThreshold = 5000;
  AcXLowerThreshold = 5000;
  AcXAverageThreshold = 700;
  
  timeBtwPksThreshold = 500;
  GyzThreshold = 8000;
}

void loop() {
  // put your main code here, to run repeatedly:

  Wire.beginTransmission(mpu);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu, 14, true);
  
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  Tmp = Wire.read()<<8|Wire.read();
  GyX = Wire.read()<<8|Wire.read();
  GyY = Wire.read()<<8|Wire.read();
  GyZ = Wire.read()<<8|Wire.read();
  
  if(AcxFirstLoop == true) {
      AcxFirstLoop = false;
      AcXInitialAcceleration = AcX;
  } 

  // Feature 1: Aerage time needed to crossover and back wrt the starting val (neutral axis)
  // If the average of the array is below the threshold, the motion is classified as shuffling 
  
  // If the current acceleration of x is above the upper threshold, and AcXDirection is set to true
  if (AcX > AcXInitialAcceleration + AcXUpperThreshold && AcXDirection == true) {
      // Set AcXCurrentTime to the current time
      AcXCurrentTime = time;
      AcXDirection = false;
      if (AcXFirstCrossover == true) {
          AcXFirstCrossover = false;
      } else {
          // Find the difference between AcXLastTime and AcXCurrentTime
          // Set AcXLastTime to AcXCurrentTime, increment AcXCycleCount, and set up as false
          AcXCycleLengths[AcXCycleCount % 2] = AcXCurrentTime - AcXLastTime ;
          AcXCycleCount++;
          AcXLastTime = AcXCurrentTime;
      }
  //Set the direction as going up
  } else if (AcX < AcXInitialAcceleration - AcXLowerThreshold) {
      AcXDirection = true;
  }
  
  
  //Feature 2: Finding Time Between Peaks in the GyZ signal
  //GyZ signal must be larger than the thre
  if (GyZ > GyzThreshold && GyzWithinTrough == true) {
      timeBtwPks = time - GyZDownTime;
      GyzWithinTrough = false;
      Serial.println("Time at Start");
  } else if (GyZ < GyzThreshold && GyzWithinTrough == false) {
      GyZDownTime = time;
      GyzWithinTrough = true;
      Serial.println("Time at End");
  }
  
  //If there have been at least 3 cycles, take the AcXAverage of the array
  if (AcXCycleCount >= 2) {
      AcXAverage = (AcXCycleLengths[0] + AcXCycleLengths[1]) / 2 ;
      if(AcXAverage < AcXAverageThreshold && timeBtwPks < timeBtwPksThreshold)
      //if(timeBtwPks < timeBtwPksThreshold)
      {
          Serial.print("Shuffle ");
          Serial.println(time);
          digitalWrite(11,HIGH);
          for(int i=0;i<5;i++)
          {
            servo.write(0);
            delay(200);
            servo.write(90);
            delay(200);
          }
          digitalWrite(11,LOW);
          delay(2000);
          time = time + 2000;
          //clear AcXCycleLengths
          AcXCycleLengths[0] = 0;
          AcXCycleLengths[1] = 0;
          AcXCycleLengths[2] = 0;
          // reset variables as necessary
          AcXFirstCrossover = true;
          AcXCycleCount = 0;
      }
  } 
  delay(50);
  time += 50;
  //Sampling rate is 20Hz
}


