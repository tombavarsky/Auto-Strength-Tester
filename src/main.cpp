#include <Arduino.h>
#include <EnableInterrupt.h>

int motor_power = 0;
float force_err = 0;
volatile long encoder_val = 0;

const int MOTOR_PWM_PIN = 10;
const int MOTOR_DIR_PIN = 12;
const int ENCODER_PIN_A = 3;
const int ENCODER_PIN_B = 5;

void encoderCount()
{
  static int lastEncoded = 0;

  int EncoderPhaseA = digitalRead(ENCODER_PIN_A); // MSB
  int EncoderPhaseB = digitalRead(ENCODER_PIN_B); // LSB

  int currentEncoded = (EncoderPhaseA << 1) | EncoderPhaseB;
  int sum = (lastEncoded << 2) | currentEncoded;
  switch (sum)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    encoder_val--;
    break;
  case 0b0010:
  case 0b1011:
  case 0b1101:
  case 0b0100:
    encoder_val++;
    break;
  }
  lastEncoded = currentEncoded;
}

void setup()
{
  Serial.begin(115200);
  // Serial.setTimeout(5);

  pinMode(13, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  enableInterrupt(ENCODER_PIN_A, encoderCount, CHANGE);
  enableInterrupt(ENCODER_PIN_B, encoderCount, CHANGE);
}

void loop()
{
  static float force_val = 0;
  static int curr_iteration = 0;
  static float wanted_force = 0;
  static int iterations = 0;
  static bool new_iteration = false;
  static const int THRESH = 0;
  static const float KP = 10;

  // while (true)
  // {
  //   Serial.println(encoder_val);
  // }

  while (wanted_force == 0 || iterations == 0)
  {
    if (Serial.available() > 2)
    {
      if (iterations == 0)
      {
        iterations = Serial.parseInt();
      }
      wanted_force = Serial.parseInt();
    }
  }

  if (Serial.available() > 0)
  {
    force_val = abs(Serial.parseFloat());

    // Serial.write((int)force_val); // for debug
  }

  // calculating motor's power
  force_err = wanted_force - force_val;
  motor_power = max(min(force_err * KP, 255), -255);

  Serial.write(encoder_val);

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