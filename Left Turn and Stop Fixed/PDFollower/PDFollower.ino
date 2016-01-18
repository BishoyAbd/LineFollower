#include "LineFollower.h"
#include <SoftwareSerial.h>
#define led 13

using namespace pdlf;

byte s[] = {61, 60, 59, 58, 57, 56, 55, 54};
byte lm[] = { 4, 5 };
byte rm[] = { 2, 3 };
//Put m for mega
// and u for uno
char mcu = 'm';

/*
 * Left and Right IR subroutines
 * 
 * 
 */
//left and right sensor
byte leftIr = 62;
byte rightIr = 63;

const int tcrt_threshold = 500;
bool tcrt_inverse_logic = false;

int left_reading = 0;
int right_reading = 0;

//Returns digital reading from threshold
int leftDigitalRead(void){
  if (!tcrt_inverse_logic){
    if (analogRead(leftIr) > tcrt_threshold) {left_reading = 1; return 1;}
    else {left_reading = 0; return 0;}
  } else {
    if (analogRead(leftIr) < tcrt_threshold) {left_reading = 0 ; return 0;}
    else {left_reading = 1; return 1;}
  }
}

//Returns digital reading from threshold
int rightDigitalRead(void){
  if (!tcrt_inverse_logic){
    if (analogRead(leftIr) > tcrt_threshold) {right_reading = 1; return 1;}
    else {right_reading = 0; return 0;}
  } else {
    if (analogRead(leftIr) < tcrt_threshold) {right_reading = 0 ; return 0;}
    else {right_reading = 1; return 1;}
  }
}

//Updates current reading of irs
void updateIr(void){
  left_reading = leftDigitalRead();
  right_reading = rightDigitalRead();
}

//Debug left and right sensor
void debugIr(void){
  Serial.println("====== BEGIN =======");
  Serial.print("LEFT : ");
  Serial.println(analogRead(leftIr));
  Serial.print("RIGHT : ");
  Serial.println(analogRead(rightIr));
  Serial.println("====== END =======");
}

//Setting up irs
void setupIr(void){
  pinMode(leftIr, INPUT);
  pinMode(rightIr, INPUT);
}


Robot *robot;

void setup()
{
	Serial.begin(9600);
	// robot.printAnalog();
	// robot.initializeComponents();
	// robot.updateWeightedValue();
	// robot.generateWeight(8);
	// robot.printWeight();
	// robot.printDigital();
	// robot.printWeightedValue();
  
	robot = new Robot(lm, rm, s, 8, mcu);
	robot->initializeComponents();
	Robot::THRESHOLD = 600;
	robot->generateWeight(8);
	
	robot->setKp(10);
	robot->setKd(1);
  pinMode(led, OUTPUT);

  setupIr();
}

void blink(int times){
  for (int i = 0; i < times; i++){
    digitalWrite(led, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    delay(200);
  }
}

void loop()
{
	
	// Serial.println("----------------------");
	// robot->printAnalog();
	// Serial.println("\n--------------------");
	// Serial.println();
	// Serial.println();
	// robot->printDigital();


	
	// delay(750);
	// robot.pidLineFollow();
	//analogWrite(lm[0], 150);
	//analogWrite(lm[1], 0);
	//analogWrite(rm[0], 150);
	//analogWrite(rm[1], 0);
/// Setting constants on the go

  // robot.printAnalog();
//	robot->pdLineFollow();

//  if(robot->weightedValueSum() >= -9 && robot->weightedValueSum() <= -7){
//    robot->run(Robot::Nowhere);
//    delay(5000);
//  } else {
//    robot->pdLineFollow();
//    delayMicroseconds(1);

//    if (robot->s_digital_reading[7] == 1 && robot->s_digital_reading[3] == 1){
//      robot->updateDigitalRead();
//    }
//  }
  
//	robot->updateWeightedValue();
//	robot->printWeightedValue();

  robot->updateDigitalRead();

  if (robot->s_digital_reading[0] == 1 && robot->s_digital_reading[3] == 1){
    robot->updateDigitalRead();
    while(robot->s_digital_reading[6] != 1){
//      robot->updateDigitalRead();
      robot->run(0, 120, Robot::Forward, Robot::Forward);
      delay(1);
      robot->updateDigitalRead();
    }
  } 
//      else if (robot->s_digital_reading[4] == 1 && robot->s_digital_reading[7] == 1){
//            robot->updateDigitalRead();
//        while(robot->s_digital_reading[2] != 1){
//          robot->run(120, 0, Robot::Forward, Robot::Forward);
//          delay(1);
//          robot->updateDigitalRead();
//        }
//      }

//   robot->updateDigitalRead();
//   if (robot->s_digital_reading[5] == 1 && 
//        robot->s_digital_reading[6] == 0 &&
//        robot->s_digital_reading[7] == 1){
//          robot->run(Robot::Nowhere);
//          delay(500); 
//          blink(10);
//        }

    else if (robot->s_digital_reading[3] == 0 &&
             robot->s_digital_reading[0] == 1 && 
             robot->s_digital_reading[7] == 1){
              robot->run(Robot::Nowhere);
              blink(10);
             }

    else {
    robot->pdLineFollow();
  }

//  robot->run(0, 150, Robot::Forward, Robot::Forward);
//robot->printDigital();
//delay(1000);
//  Serial.println("Weighted val: " + String(robot->weightedValueSum()));
// delay(1000);
}
