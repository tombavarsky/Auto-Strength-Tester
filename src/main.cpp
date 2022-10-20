#include <Arduino.h>

int motor_power = 0;
float force_err = 0;
volatile long encoder_val = 0;

const int MOTOR_PWM_PIN = 10;
const int MOTOR_DIR_PIN = 12;
const int ENCODER_PIN = 3;

void update_encoder() { encoder_val++; }

void setup()
{
  Serial.begin(115200);
  // Serial.setTimeout(5);

  pinMode(13, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), update_encoder, RISING);
}

void loop()
{
  static float force_val = 0;
  static int curr_iteration = 0;
  static float wanted_force = 0;
  static int iterations = 0;
  static bool new_iteration = false;
  static const int THRESH = 0;
  static const float KP = 15;

  while (wanted_force == 0 || iterations == 0)
  {
    if (Serial.available() > 2)
    {
      if (iterations == 0)
      {
        iterations = Serial.parseInt();
      }
      wanted_force = Serial.parseInt();
      // Serial.write("---i---");
      // Serial.write(int(iterations));
      // Serial.write("---w---");
      // Serial.write(int(wanted_force));
    }

    // Serial.print(iterations);
    // Serial.print("   ");
    // Serial.print(Serial.available());
    // Serial.print("   ");
    // Serial.println(wanted_force);
  }

  if (Serial.available() > 0)
  {
    // Serial.println("------------------------------");
    force_val = abs(Serial.parseFloat());

    // Serial.write((int)force_val); // for debug
  }

  // calculating motor's power
  force_err = wanted_force - force_val;
  motor_power = max(min(force_err * KP, 255), -255);

  Serial.write(abs(motor_power));
  if (motor_power < -THRESH)
  {
    digitalWrite(MOTOR_DIR_PIN, 0);
  }
  else if (motor_power > THRESH)
  {
    digitalWrite(MOTOR_DIR_PIN, 1);
  }

  analogWrite(MOTOR_PWM_PIN, abs(motor_power));

  // Serial.write(int(force_err));

  if (force_err < 1 && force_err > -1 && !new_iteration)
  { // reached wanted force and therefore finnished the iteration.
    curr_iteration++;
    Serial.write(int(curr_iteration));
    new_iteration = true;
  }
  if (new_iteration && force_val == 0)
  {
    new_iteration = false;
  }

  if (curr_iteration == iterations)
  {
    Serial.write("finfinfin");

    iterations = 0;
    wanted_force = 0;
    while (true)
    {
      analogWrite(MOTOR_PWM_PIN, 0);
    }
  }
}