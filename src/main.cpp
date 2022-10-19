#include <Arduino.h>

int motor_power = 0;
float force_err = 0;
const float KP = 1;
const int MOTOR_PIN = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
}

void loop()
{
  static float force_val = 0;
  static int curr_iteration = 0;
  static float wanted_force = 0;
  static int iterations = 0;
  static bool new_iteration = false;

  while (wanted_force == 0 || iterations == 0)
  {
    if (Serial.available() > 1)
    {
      iterations = Serial.read();
      wanted_force = Serial.read();
    }
  }
  // Serial.write(int(force_err));

  if (Serial.available() > 0)
  {
    force_val = abs(Serial.parseFloat());

    // Serial.write((int)force_val); // for debug
  }

  force_err = wanted_force - force_val;
  motor_power = force_err * KP;
  analogWrite(MOTOR_PIN, motor_power);

  // Serial.write(iterations);
  if (force_err < 2 && force_err > -2 && !new_iteration)
  { // reached wanted force and therefore finnished the iteration.
    curr_iteration++;
    Serial.write(int(curr_iteration));
    Serial.write("----");
    Serial.write(int(iterations));
    new_iteration = true;
  }
  if (new_iteration && force_val == 0)
  {
    new_iteration = false;
  }

  if (curr_iteration == iterations)
  {
    Serial.write("fin");
  }
}