#include <EEPROMex.h>
#include <QTRSensors.h>

int addrCalibratedMinimumOn = 0;
int addrCalibratedMaximumOn = 50;

#define Kp_lfr 1249
#define Kd_lfr 400

// PID gains
const float K_p = 30;//1
const float K_i = 20.5;//0.0075
/*
const int trig = 39;
const int echo = 38;
unsigned long pulse_us;
int distance;

const int targetDistance = 109; // target distance from wall is 152 mm
int e; // the error (target distance - measured distance)
int u; // the PID output
int p = 0;
float i = 0;
*/

#define rightMaxSpeed 255
#define leftMaxSpeed 255
#define rightBaseSpeed 250
#define leftBaseSpeed 250

#define NUM_SENSORS 8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN 10

const int thresh = 600;

QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
}
,
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];


//motor code
#define leftMotor_inputC 3   //any digital pin
#define leftMotor_inputD  2  //any digital pin

#define rightMotor_inputA 5  //any digital pin
#define rightMotor_inputB 4  //any digital pin

#define rightMotor_enableA  6 //Use for PWM Pn
#define leftMotor_enableB  7 //Use for PWM Pin



int lm;
int rm;

int rightMotorSpeed_lfr;
int leftMotorSpeed_lfr;

/*
int leftMotorSpeed;
int rightMotorSpeed;
*/
void setup()
{
  //setup for Motor Input and Speed Control
  pinMode (rightMotor_inputA, OUTPUT);
  pinMode (rightMotor_inputB, OUTPUT);

  pinMode (leftMotor_inputC, OUTPUT);
  pinMode (leftMotor_inputD, OUTPUT);

  //PWM OUTPUT
  pinMode (rightMotor_enableA, OUTPUT);
  pinMode (leftMotor_enableB, OUTPUT);

  qtra.calibrate();
  EEPROM.readBlock(addrCalibratedMinimumOn, qtra.calibratedMinimumOn, 8);
  EEPROM.readBlock(addrCalibratedMaximumOn, qtra.calibratedMaximumOn, 8);

  pinMode(11, OUTPUT);
  //sonar sensor
 /*
  pinMode (trig, OUTPUT);
  pinMode (echo, INPUT);
*/

}


void loop ()
{
 lfr ();
  qtra.readLine(sensorValues);
  reading ();
  //getDistance();
  /*

    //for wall follow
    if (distance>10 && distance <180)
    {
    motorStop ();
    delay (10);
    //buzz ();
    // buzz ();
    //buzz ();
    Forward ();
    delay (200);
    distance=0;
    motorStop ();
    delay (10);
    // buzz ();
    while (1)
    {
      getDistance();
      followWall();
      qtra.readLine(sensorValues);
      reading ();
      if (sensorValues [7]>=thresh|| sensorValues [6]>=thresh || sensorValues [5]>=thresh || sensorValues [4]>=thresh || sensorValues [3]>=thresh || sensorValues [2]>=thresh || sensorValues [1]>=thresh || sensorValues [0]>=thresh)
      {
        motorStop ();
        delay (10);
     //   buzz ();
        Forward ();
        delay (200);
        motorStop ();
        delay (10);
        break;
        }
     }
     distance=0;
    }
  */


  // for + condition
  if (lm >= thresh && rm >= thresh )
  {
    motorStop ();
    delay (500);
    buzz ();
    buzz ();
    buzz ();
    buzz ();
    buzz ();
    while (1)
    {
      Forward ();
      qtra.readLine(sensorValues);
      if (sensorValues [6] >= thresh && sensorValues [5] >= thresh && sensorValues [4] >= thresh && sensorValues [3] >= thresh && sensorValues [2] >= thresh)
      {
        motorStop ();
        delay (1000);
        Forward2 ();
        delay (30);
        buzz ();
        break;

      }
    }
    lm = 0;
    rm = 0;
  }




  //for 90 degree right

  if (rm >= thresh )
  {
    motorStop ();
    delay (10);
 //   buzz ();
 //   buzz ();
 //   buzz ();
    Forward ();
    delay (180);
    hardRight ();
    rm=0;
   /* while (1)
    {
      Forward ();
      qtra.readLine(sensorValues);
      if (sensorValues [6] >= thresh && sensorValues [5] >= thresh && sensorValues [4] >= thresh && sensorValues [3] >= thresh)
      {
        motorStop ();
        delay (1000);
        Forward2 ();
        delay (50);
        buzz ();
        break;

      }

      else if ( sensorValues [0] >= thresh)
      {
        motorStop ();
        delay (1000);
        Forward2 ();
        delay (30);
        buzz ();
        hardRight ();
        break;

      }
    }
    rm = 0;
    */
  }


  //for 90 degree left

  if (lm >= thresh )
  {
    motorStop ();
    delay (10);
 //   buzz ();
  //  buzz ();
   // buzz ();
    Forward ();
    delay (150);
    hardLeft ();
    lm=0;
   /* while (1)
    {
      Forward ();
      qtra.readLine(sensorValues);
      if (sensorValues [6] >= thresh && sensorValues [5] >= thresh && sensorValues [4] >= thresh && sensorValues [3] >= thresh)
      {
        motorStop ();
        delay (1000);
        Forward2 ();
        delay (30);
        buzz ();
        break;

      }

      else if (sensorValues [7] >= thresh)
      {
        motorStop ();
        delay (1000);
        Forward2 ();
        delay (30);
        buzz ();
        hardLeft ();
        break;

      }
    }
    lm = 0;
  }
*/
  }

  /*
//for Ending
if (lm>=thresh && rm >=thresh && (sensorValues [4] >= thresh || sensorValues [5] >= thresh || sensorValues [3] >= thresh ))
{
  motorStop ();
  delay (5000);
  while (1)
  {
    buzz ();
    }
  }
*/
}

void hardLeft ()
{
  while (1)
  {
    leftTurn ();
    reading ();
    if (lm >= thresh)
    {
      while (1)
      {
        leftTurn ();
        qtra.readLine(sensorValues);
        if ( sensorValues [4] >= thresh && sensorValues [5] >= thresh)
        {
          motorStop ();
          delay (10);
          //     buzz ();
          //delay (50);
          break;

        }
      }
      break;
    }
  }

}

void hardRight ()
{
  while (1)
  {
    rightTurn ();
    reading ();
    if (rm >= thresh)
    {
      while (1)
      {
        rightTurn ();
        qtra.readLine(sensorValues);
        if ( sensorValues [2] >= thresh && sensorValues [1] >= thresh)
        {
          motorStop ();
          delay (10);
          //   buzz ();
          //delay (50);
          break;

        }
      }
      break;
    }
  }
}


void reading ()
{
  lm = analogRead (A9);
  rm = analogRead (A8);
}
/*
void getDistance () {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long pulse_us = pulseIn(echo, HIGH);
  distance = (pulse_us / 2) / 3;
}

void followWall () {


  // int oldE = e;

  e = targetDistance - distance;

  p = K_p * e; // p gets bigger when the wall is too close and smaller when it is too far away
  i += K_i * e; // i gets bigger when the wall is too close and smaller when it is too far away

  // limit p
  if (p > 200)
    p = 200;
  else if (p < -200)
    p = -200;

  // limit i
  if (i > 200)
    i = 200;
  else if (i < -200)
    i = -200;

  u = p + i;

  leftMotorSpeed  = leftMaxSpeed + u;
  rightMotorSpeed = rightMaxSpeed - u;

  // limit leftMotorSpeed
  if (leftMotorSpeed > 255)
    leftMotorSpeed = 200;
  else if (leftMotorSpeed < 0)
    leftMotorSpeed = 0;

  // limit rightMotorSpeed
  if (rightMotorSpeed > 255)
    rightMotorSpeed = 200;
  else if (rightMotorSpeed < 0)
    rightMotorSpeed = 0;

  Forward_wall ();

}

void Forward_wall ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA, rightMotorSpeed );
  analogWrite (leftMotor_enableB, leftMotorSpeed );
}//end of Forward Function

*/
int lastError_array = 0;


void lfr ()
{
  int position = qtra.readLine(sensorValues);
  delay (8);

  int error_lfr = (position - 3200) / 1000;
  int motorSpeed_lfr = Kp_lfr * error_lfr + Kd_lfr * (error_lfr - lastError_array);

  lastError_array = error_lfr;

  rightMotorSpeed_lfr = rightBaseSpeed + motorSpeed_lfr;
  leftMotorSpeed_lfr = leftBaseSpeed - motorSpeed_lfr;

  if (rightMotorSpeed_lfr > rightMaxSpeed )
  {
    rightMotorSpeed_lfr = rightMaxSpeed;
  }

  if (leftMotorSpeed_lfr > leftMaxSpeed )
  {
    leftMotorSpeed_lfr = leftMaxSpeed;
  }

  if (rightMotorSpeed_lfr < 0)
  {
    rightMotorSpeed_lfr = 0;
  }

  if (leftMotorSpeed_lfr < 0)
  {
    leftMotorSpeed_lfr = 0;
  }
  {
    Forward_lfr ();
  }

}


void Forward_lfr ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA, rightMotorSpeed_lfr );
  analogWrite (leftMotor_enableB, leftMotorSpeed_lfr );
}//end of Forward Function


void Forward ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA, 80);
  analogWrite (leftMotor_enableB, 80);

}


void Forward2 ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA, 80); //suhdu pwm er jonno
  analogWrite (leftMotor_enableB, 80);
}//end of Forward Function
/*
  void Forward_3 ()
  {
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA,180);//suhdu pwm er jonno
  analogWrite (leftMotor_enableB,180);

  void Forward_4 ()
  {
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA,180);//suhdu pwm er jonno
  analogWrite (leftMotor_enableB,180);
  }//end of Forward Function

*/

void rightTurn ()
{
  //Left Motor Backward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, LOW);
  digitalWrite (rightMotor_inputB, HIGH);


  analogWrite (rightMotor_enableA, 50);//40
  analogWrite (leftMotor_enableB, 100 );//80
}

/*
  void rightTurn_2 ()
  {
  //Left Motor Backward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, LOW);
  digitalWrite (rightMotor_inputB, HIGH);


  analogWrite (rightMotor_enableA,255);//eikhaner jonno
  analogWrite (leftMotor_enableB,255 );

  }


  void rightTurn_3 ()
  {
  //Left Motor Backward
  digitalWrite (leftMotor_inputC, HIGH);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, LOW);
  digitalWrite (rightMotor_inputB, HIGH);


  analogWrite (rightMotor_enableA,255);//eikhaner jonno
  analogWrite (leftMotor_enableB,255 );

  }

*/

void leftTurn ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, LOW);
  digitalWrite (leftMotor_inputD, HIGH);

  //Right Motor Backward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA, 100 );//80
  analogWrite (leftMotor_enableB, 55);//45
}

/*
  void leftTurn_2 ()
  {
   //Left Motor Forward
  digitalWrite (leftMotor_inputC, LOW);
  digitalWrite (leftMotor_inputD, HIGH);

  //Right Motor Backward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA,255 );
  analogWrite (leftMotor_enableB,255);//eikhane

  }

  void leftTurn_3 ()
  {
   //Left Motor Forward
  digitalWrite (leftMotor_inputC, LOW);
  digitalWrite (leftMotor_inputD, HIGH);

  //Right Motor Backward
  digitalWrite (rightMotor_inputA, HIGH);
  digitalWrite (rightMotor_inputB, LOW);

  analogWrite (rightMotor_enableA,255 );
  analogWrite (leftMotor_enableB,255);//eikhane

  }

*/
/*
  void back ()
  {
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, LOW);
  digitalWrite (leftMotor_inputD, HIGH);

  //Right Motor Forward
  digitalWrite (rightMotor_inputA, LOW);
  digitalWrite (rightMotor_inputB, HIGH);

  analogWrite (rightMotor_enableA,180);
  analogWrite (leftMotor_enableB,180);
  }//end of Forward Function
*/

void buzz() {
  analogWrite(11, 255);      // Almost any value can be used except 0 and 255
  // experiment to get the best tone
  delay(200);          // wait for a delayms ms
  analogWrite(11, 0);       // 0 turns it off
  delay(200);          // wait for a delayms ms
}


void motorStop ()
{
  //Left Motor Forward
  digitalWrite (leftMotor_inputC, LOW);
  digitalWrite (leftMotor_inputD, LOW);

  //Right Motor Backward
  digitalWrite (rightMotor_inputA, LOW);
  digitalWrite (rightMotor_inputB, LOW);
}
