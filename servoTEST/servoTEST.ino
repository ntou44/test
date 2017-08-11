/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMAXNum  16 // 
#define ServoSpeed  1

// our servo # counter
uint8_t servonum = 0;

int ServerMax[SERVOMAXNum] = {SERVOMAX, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int ServerMin[SERVOMAXNum] = {SERVOMIN, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int ServerDirection[SERVOMAXNum] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int ServerCurrentPoint[SERVOMAXNum] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
//int ServerSpeed[SERVOMAXNum] = {1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int ServerWaitTime[SERVOMAXNum] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int ServerCurrentWaitTime[SERVOMAXNum] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

void setup() {
  Serial.begin(115200);
  Serial.println("16 channel Servo test!");

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();

  int space = 100;
  for (uint16_t i = 1; i < SERVOMAXNum; i++)
  {
//    ServerMax[i] = ServerMax[i - 1] - space;
//    ServerMin[i] = ServerMin[i - 1] + space;
    ServerMax[i] = SERVOMAX;
    ServerMin[i] = SERVOMIN;
    //ServerCurrentPoint [i - 1] = ServerMin[i - 1];
    ServerCurrentPoint [i] = SERVOMIN;

    ServerWaitTime[i] =  ServerWaitTime[i-1]+1;
    ServerCurrentWaitTime[i] = ServerWaitTime[i];
  }
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

bool CheckServerDelay(int servoIndex) {
  ServerCurrentWaitTime[servoIndex] -= 1;
  bool isServoCanMove = false;


  //Serial.print("ServerCurrentWaitTime=");
  //Serial.print( ServerCurrentWaitTime[servoIndex], DEC);
  if ( ServerCurrentWaitTime[servoIndex] <= 0)
  {
  //Serial.print(" ServerWaitTime=");
  //Serial.print( ServerWaitTime[servoIndex], DEC);
   // Serial.println(" move");
    isServoCanMove = true;
    ServerCurrentWaitTime[servoIndex] = ServerWaitTime[servoIndex];

  }  


  //Serial.println("  "); 
  
  return isServoCanMove;
}

void MoveServer(int servoIndex) {

  int moveSpeed = 3;

  if (!CheckServerDelay(servoIndex))
  {
    
   // Serial.println(" pass");
    return;
    }

  if (ServerDirection[servoIndex] == 0)
  {
    //+
    ServerCurrentPoint [servoIndex] = ServerCurrentPoint[servoIndex] + moveSpeed;

  } else
  {
    //-
    ServerCurrentPoint [servoIndex] = ServerCurrentPoint[servoIndex] - moveSpeed;
  }

  if (ServerDirection[servoIndex] == 0 && (ServerCurrentPoint [servoIndex] > ServerMax[servoIndex]))
  {
    //到最大
    ServerDirection[servoIndex] = 1;
    ServerCurrentPoint [servoIndex] = ServerMax[servoIndex] - 1;

  } else  if (ServerDirection[servoIndex] == 1 && (ServerCurrentPoint [servoIndex] < ServerMin[servoIndex]))
  {
    //到最小
    ServerDirection[servoIndex] = 0;
    ServerCurrentPoint [servoIndex] = ServerMin[servoIndex] + 1;
  }

  pwm.setPWM(servoIndex, 0, ServerCurrentPoint[servoIndex]);

  //Serial.print("i=");
 // Serial.print(servoIndex, DEC);
 // Serial.print(" Point=");
 // Serial.println(ServerCurrentPoint[servoIndex], DEC); 
}


void loop() {

  for (uint16_t i = 0; i < SERVOMAXNum; i++)
  {
     MoveServer(i);
  }
  
     //MoveServer(4);
//delay(ServoSpeed);

  
//  // Drive each servo one at a time
//  Serial.println(servonum);
//  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
//    pwm.setPWM(servonum, 0, pulselen);
//    delay(ServoSpeed);
//  }
//
//  delay(500);
//  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
//    pwm.setPWM(servonum, 0, pulselen);
//    delay(ServoSpeed);
//  }
//
//  delay(500);
//
//  servonum ++;
//  if (servonum > 0) servonum = 0;
  //if (servonum > 7) servonum = 0;
}
