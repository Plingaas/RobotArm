#include "RobotArmController.hpp"

RobotArm::RobotArm()
{

  Position pos;
  acc_time_percentage = 0.2;
  dec_time_percentage = 0.2;

  delay_after_move = 100;

  speed = 0;        // steps / second
  acceleration = 0; // steps / second^2
  useJerk = false;
  arduinoPin = 30;
  pinMode(arduinoPin, OUTPUT);

  actuators[0].lower_limit = J1_LOWER_LIMIT;
  actuators[0].upper_limit = J1_UPPER_LIMIT;
  actuators[0].steps_per_deg = J1_STEPS_PER_DEG;
  actuators[1].lower_limit = J2_LOWER_LIMIT;
  actuators[1].upper_limit = J2_UPPER_LIMIT;
  actuators[1].steps_per_deg = J2_STEPS_PER_DEG;
  actuators[2].lower_limit = J3_LOWER_LIMIT;
  actuators[2].upper_limit = J3_UPPER_LIMIT;
  actuators[2].steps_per_deg = J3_STEPS_PER_DEG;
  actuators[3].lower_limit = J4_LOWER_LIMIT;
  actuators[3].upper_limit = J4_UPPER_LIMIT;
  actuators[3].steps_per_deg = J4_STEPS_PER_DEG;
  actuators[4].lower_limit = J5_LOWER_LIMIT;
  actuators[4].upper_limit = J5_UPPER_LIMIT;
  actuators[4].steps_per_deg = J5_STEPS_PER_DEG;
  actuators[5].lower_limit = J6_LOWER_LIMIT;
  actuators[5].upper_limit = J6_UPPER_LIMIT;
  actuators[5].steps_per_deg = J6_STEPS_PER_DEG;
}

void RobotArm::update(float dt)
{

  for (int i = 0; i < 6; i++)
  {
    actuators[i].update(dt);
  }
}

void RobotArm::setTarget(Vector3 _target)
{

  Angles angles = IK(_target.x, _target.y, _target.z);

  target = _target;
  actuators[0].setTarget(angles.theta1 * TO_DEGREES * J1_STEPS_PER_DEG);
  actuators[1].setTarget(angles.theta2 * TO_DEGREES * J2_STEPS_PER_DEG);
  actuators[2].setTarget(angles.theta3 * TO_DEGREES * J3_STEPS_PER_DEG);
  actuators[3].setTarget(angles.theta4 * TO_DEGREES * J4_STEPS_PER_DEG);
  actuators[4].setTarget(angles.theta5 * TO_DEGREES * J5_STEPS_PER_DEG);
  actuators[5].setTarget(angles.theta6 * TO_DEGREES * J6_STEPS_PER_DEG);
}

void RobotArm::goToWithSpeed(float x, float y, float z, float rx, float ry, float rz)
{
  Angles angles = IK(x, y, z);

  if (z < 50)
  {
    // Serial.println("ERROR: POSITION IS OUTSIDE OF WORKSPACE");
    return;
  }

  int j1Steps = angles.theta1 * J1_STEPS_PER_DEG;
  int j2Steps = angles.theta2 * J2_STEPS_PER_DEG;
  int j3Steps = angles.theta3 * J3_STEPS_PER_DEG;

  short max_steps = j1Steps; // needs more work
  float move_time = max_steps / speed;

  actuators[0].moveTo(j1Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[1].moveTo(j2Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[2].moveTo(j3Steps, move_time, acc_time_percentage, dec_time_percentage);

  run();
}

void RobotArm::goToWithTime(float x, float y, float z, float move_time, const Matrix3 &orientation)
{

  if (z < 0)
  {
    return;
  }

  if (useJerk)
  {
    goToWithTime2(x, y, z, move_time, orientation);
    return;
  }

  Angles angles = IK(x, y, z, orientation);

  int j1Steps = angles.theta1 * TO_DEGREES * J1_STEPS_PER_DEG;
  int j2Steps = angles.theta2 * TO_DEGREES * J2_STEPS_PER_DEG;
  int j3Steps = angles.theta3 * TO_DEGREES * J3_STEPS_PER_DEG;
  int j4Steps = angles.theta4 * TO_DEGREES * J4_STEPS_PER_DEG;
  int j5Steps = angles.theta5 * TO_DEGREES * J5_STEPS_PER_DEG;
  int j6Steps = angles.theta6 * TO_DEGREES * J6_STEPS_PER_DEG;

  actuators[0].moveTo(j1Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[1].moveTo(j2Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[2].moveTo(j3Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[3].moveTo(j4Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[4].moveTo(j5Steps, move_time, acc_time_percentage, dec_time_percentage);
  actuators[5].moveTo(j6Steps, move_time, acc_time_percentage, dec_time_percentage);

  run();
}

void RobotArm::goToWithTime2(float x, float y, float z, float move_time, const Matrix3 &orientation)
{
  if (z < 0)
  {
    return;
  }

  Angles angles = IK(x, y, z, orientation);

  int j1Steps = angles.theta1 * TO_DEGREES * J1_STEPS_PER_DEG;
  int j2Steps = angles.theta2 * TO_DEGREES * J2_STEPS_PER_DEG;
  int j3Steps = angles.theta3 * TO_DEGREES * J3_STEPS_PER_DEG;
  int j4Steps = angles.theta4 * TO_DEGREES * J4_STEPS_PER_DEG;
  int j5Steps = angles.theta5 * TO_DEGREES * J5_STEPS_PER_DEG;
  int j6Steps = angles.theta6 * TO_DEGREES * J6_STEPS_PER_DEG;

  actuators[0].moveToJerk(j1Steps, 500, 500, move_time);
  actuators[1].moveToJerk(j2Steps, 400, 400, move_time);
  actuators[2].moveToJerk(j3Steps, 500, 500, move_time);
  actuators[3].moveToJerk(j4Steps, 500, 500, move_time);
  actuators[4].moveToJerk(j5Steps, 400, 400, move_time);
  actuators[5].moveToJerk(j6Steps, 500, 500, move_time);

  run2();
}

void RobotArm::goToHome(float t)
{
  goToWithTime(692, 0, 169.77, t, LOOKFORWARD);
}

void RobotArm::goToRest(float t)
{
  goToWithTime(64.2, 0, 797.6, t, LOOKUP); // 697.4, t);
}

void RobotArm::goToAngles(float (&angles)[6])
{
  for (int i = 0; i < 6; i++)
  {
    actuators[i].moveToAngle(angles[i], 5, 0.2, 0.2);
  }

  run();
}

void RobotArm::run2()
{

  bool a, b, c, d, e, f = false;
  bool finished = false;

  int index = 0;

  unsigned long last = micros();
  unsigned long now = micros();
  while (!finished)
  {

    now = micros();

    a = actuators[0].run2(now - last);
    b = actuators[1].run2(now - last);
    c = actuators[2].run2(now - last);
    d = actuators[0].run2(now - last);
    e = actuators[1].run2(now - last);
    f = actuators[2].run2(now - last);

    if ((now - last) * 1e-6 * 5 > index)
    {
      index++;
      Serial.print(actuators[0].total_steps - abs(actuators[0].steps_moved));
      Serial.print(",");

      Serial.print(actuators[1].total_steps - abs(actuators[1].steps_moved));
      Serial.print(",");

      Serial.print(actuators[2].total_steps - abs(actuators[2].steps_moved));
      Serial.print(",");
      Serial.println();
    }

    finished = a && b && c;
  }

  Serial.print(actuators[0].total_steps - abs(actuators[0].steps_moved));
  Serial.print(",");

  Serial.print(actuators[1].total_steps - abs(actuators[1].steps_moved));
  Serial.print(",");

  Serial.print(actuators[2].total_steps - abs(actuators[2].steps_moved));
  Serial.print(",");
  Serial.println();

  delay(delay_after_move);
}

void RobotArm::run()
{

  bool a, b, c, d, e, f = false;
  bool finished = false;

  // elapsedMicros time_elapsed;
  float serial_tx_time = 16666.66667; // 60 per sec
  unsigned long last_tx = 0;

  // elapsedMicros time_elapsed = 0;
  unsigned long g = micros();
  while (!finished)
  {
    a = actuators[0].run(micros() - g);
    b = actuators[1].run(micros() - g);
    c = actuators[2].run(micros() - g);
    d = actuators[3].run(micros() - g);
    e = actuators[4].run(micros() - g);
    f = actuators[5].run(micros() - g);
    finished = a && b && c && d && e && f;

    if (micros() - g - last_tx > serial_tx_time)
    {
      // writePosition();
    }
  }

  delay(delay_after_move);
}

void RobotArm::writePosition()
{
  String data = "A" + (String)actuators[0].currentPosition + "B" + (String)actuators[1].currentPosition + "C" + (String)actuators[2].currentPosition;
  data += "<";

  Serial.println(data);
}

void RobotArm::splineMove(BSpline spline)
{

  Vector3 startPos = spline.getSplinePoint();
  goToWithTime(startPos.x, startPos.y, startPos.z, 5);

  unsigned long last = micros();
  unsigned long now = last;
  float dt = 0;

  float serial_tx_time = 0.2; // 5 per sec
  float txTimer = 0;

  while (spline.t < spline.tmax)
  {

    now = micros();
    dt = (now - last) * 1e-6;
    last = now;

    spline.t += dt * 0.33;
    Angles angles = IK(spline.getSplinePoint());

    Vector3 _angles = {angles.theta1 * TO_DEGREES * J1_STEPS_PER_DEG,
                       angles.theta2 * TO_DEGREES * J2_STEPS_PER_DEG,
                       angles.theta3 * TO_DEGREES * J3_STEPS_PER_DEG};

    Vector3 lastPositions = {actuators[0].currentPosition, actuators[1].currentPosition, actuators[2].currentPosition};

    for (int i = 0; i < 3; i++)
    {

      if (_angles[i] > actuators[i].currentPosition && actuators[i].direction == actuators[i].CCW)
      {
        actuators[i].setDirection(actuators[i].CW);
      }

      if (_angles[i] < actuators[i].currentPosition && actuators[i].direction == actuators[i].CW)
      {
        actuators[i].setDirection(actuators[i].CCW);
      }

      if ((int)_angles[i] - (int)lastPositions[i] != 0)
      {
        actuators[i].step();
      }
    }

    txTimer += dt;
    if (txTimer > serial_tx_time)
    {
      Serial.println((int)actuators[0].currentPosition);
      txTimer -= serial_tx_time;
    }
  }
}

void RobotArm::grip()
{
  digitalWrite(arduinoPin, HIGH);
  delay(500);
}

void RobotArm::release()
{
  digitalWrite(arduinoPin, LOW);
  delay(500);
}

void RobotArm::calibrate()
{

  // Set direction CCW
  actuators[0].setDirection(1);
  actuators[1].setDirection(1);
  actuators[2].setDirection(0);
  actuators[3].setDirection(0);
  actuators[4].setDirection(1);
  actuators[5].setDirection(0);

  // Calibrate joint 5 first because of gripper
  while (!actuators[4].readLimitSwitch())
  {
    actuators[4].step();
    delayMicroseconds(2000);
  }

  actuators[4].currentPosition = 2400;
  actuators[4].moveToAngle(0, 3, 0.3, 0.3);
  unsigned long start = micros();
  bool finished = false;
  while (!finished)
  {
    finished = actuators[4].run(micros() - start);
  }

  finished = false;

  while (!finished)
  {

    finished = true;
    for (int i = 0; i < 6; i++)
    {
      if (i == 4)
        continue;

      if (!actuators[i].readLimitSwitch())
      {
        actuators[i].step();
        finished = false;
      }
    }

    // because j1 is slow
    delayMicroseconds(667);
    if (!actuators[0].readLimitSwitch())
    {
      actuators[0].step();
      finished = false;
    }
    delayMicroseconds(667);
    if (!actuators[0].readLimitSwitch())
    {
      actuators[0].step();
      finished = false;
    }
    delayMicroseconds(667);
  }

  actuators[0].currentPosition = 15100;
  actuators[1].currentPosition = 7200;
  actuators[2].currentPosition = -8100;
  actuators[3].currentPosition = -7600;
  actuators[5].currentPosition = -3200;

  // goToHome();
}