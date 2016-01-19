#include <Motor.h>
#define QTR_THRESHOLD 600
#define SETPOINT 4500

Motor *motor;

motorPins pins = {2, 3, 5, 4};

int sensor[] = {54, 55, 56, 57, 58, 59, 60, 61};


int iMotorSpeed = 80;


float kp = .2;
float kd = .1;
float ki = .0000015;

//Init sensor
void initSensor(void){
  for (int i = 0; i < 8; i++){
    pinMode(sensor[i], INPUT);
  }
}


//error
float error = 0;
float previousError = 0;
float totalError = 0;
float power = 0;

int PWM_Left = 0;
int PWM_Right = 0;

int iLastRead;

int iReadArray(void){
  int iRead = 0;
  int iActive = 0;

  for (int i = 0; i < 8; i++){
    if (analogRead(sensor[i]) > QTR_THRESHOLD){
      iRead += (i+1) * 1000;
      iActive++;
    }
  }

  iRead = map(iRead/iActive, 0, 8000, 0, 1023);
  
  if (!iRead) return iLastRead;
  else {
    iLastRead = iRead;
    return iRead;
  }
}

void PID(void){
  int avgSensor = iReadArray();

  previousError = error;
  error = avgSensor - map(SETPOINT, 0, 8000, 0, 1023);

  totalError += error;

  power = (kp * error) + (kd*(error - previousError)) + (ki*totalError);

  Serial.println("power " + String(power));

  if (power > iMotorSpeed) { power = iMotorSpeed; }
  if (power < -1 * iMotorSpeed) { power = -1 * iMotorSpeed; }

  

  if (power < 0){
    PWM_Right = iMotorSpeed;
    PWM_Left = iMotorSpeed - abs(int(power));
  } else {
    PWM_Right = iMotorSpeed - int(power);
    PWM_Left = iMotorSpeed;
  }

  motor->go(PWM_Left, PWM_Right, FORWARD);

  Serial.println("Left speed: " + String(PWM_Left));
  Serial.println("Right speed: " + String(PWM_Right));
}


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  initSensor();
  motor = new Motor(pins);
}

void loop() {
  // put your main code here, to run repeatedly:
  PID();
//  delay(500);

}
