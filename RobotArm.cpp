#include "RobotArm.h"

RobotArm::RobotArm() 
{

  Position pos;
  acc_time_percentage = 0.2; 
  dec_time_percentage = 0.2;

  delay_after_move = 100;

  speed = 1000; // steps / second
  acceleration = 1000; // steps / second^2
}

void RobotArm::setStepperPins(short index, short stepPin, short dirPin) 
{
  actuators[index].setPins(stepPin, dirPin);
}

void RobotArm::setStepperReverse(short index, bool reverse) 
{
  actuators[index].setReverse(reverse);
}

void RobotArm::goToWithSpeed(float x, float y, float z, float rx, float ry, float rz) 
{
  Angles angles = IK(x, y, z);

  if (!angles.legal || z < 50) {
    //Serial.println("ERROR: POSITION IS OUTSIDE OF WORKSPACE");
    return;
  }
  
  int j1Steps = angles.theta1 * J1_STEPS_PER_DEG;
  int j2Steps = angles.theta2 * J2_STEPS_PER_DEG;
  int j3Steps = angles.theta3 * J3_STEPS_PER_DEG;

  short max_steps = j1Steps; // needs more work
  float move_time  = max_steps/speed;
  
  actuators[0].moveTo(j1Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[1].moveTo(j2Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[2].moveTo(j3Steps, move_time, acc_time_percentage, dec_time_percentage);
  
  run();
}

void RobotArm::goToWithTime(float x, float y, float z, float move_time)
{

  //Serial.println("STARTING MOVE");
  Angles angles = IK(x, y, z);

/*
  Serial.println("================================");
  Serial.print("J1: ");
  Serial.println(angles.theta1);
  Serial.print("J2: ");
  Serial.println(angles.theta2);
  Serial.print("J3: ");
  Serial.println(angles.theta3);
  Serial.println("--------------------------------\n");
  */

  if (!angles.legal || z < 50) {
    //Serial.println("ERROR: POSITION IS OUTSIDE OF WORKSPACE");
    return;
  }
  
  int j1Steps = angles.theta1 * 88.88888889;
  int j2Steps = angles.theta2 * 55.55555567;
  int j3Steps = angles.theta3 * 55.55555567;

  actuators[0].moveTo(j1Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[1].moveTo(j2Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[2].moveTo(j3Steps, move_time, acc_time_percentage, dec_time_percentage);
  
  run();
}

void RobotArm::goToHome() {
  goToWithTime(591.83, 0, 169.77, 5);
}

void RobotArm::goToRest() {
  goToWithTime(64.2, 0, 697.4, 5);
}

Angles RobotArm::solveIK(float x, float y, float z) 
{
  Angles angles = IK(x, y, z);

  if (!angles.legal || z < 50) {
    //Serial.println("ERROR: POSITION IS OUTSIDE OF WORKSPACE");
    return angles;
  }

  return angles;
}

void RobotArm::run() 
{

  bool a, b, c = false;
  bool finished = false;
  
  //elapsedMicros time_elapsed;
  float serial_tx_time = 16666.66667; // 60 per sec
  unsigned long last_tx = 0;

  //elapsedMicros time_elapsed = 0;
  unsigned long g = micros();
  while (!finished) 
  {
    a = actuators[0].run(micros()-g);
    b = actuators[1].run(micros()-g);
    c = actuators[2].run(micros()-g);
    finished = a && b && c;

    if (micros()-g - last_tx > serial_tx_time)
    {
      last_tx = micros()-g;
      String data = "A" + (String) actuators[0].currentPosition + "B" + (String) actuators[1].currentPosition + "C" + (String) actuators[2].currentPosition;
      data += "<";
      
      Serial.println(data);
    }
  }

  delay(delay_after_move);
}