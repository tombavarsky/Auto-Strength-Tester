#include <Arduino.h>
#include <EnableInterrupt.h>
#include <PID_v1.h>
#include <ArduinoJson.h>

float force_err = 0;
volatile long encoder_val = 0;

const int MOTOR_PWM_PIN = 10;
const int MOTOR_DIR_PIN = 12;
const int ENCODER_PIN_A = 3;
const int ENCODER_PIN_B = 5;

const byte buffSize = 40;
char messageFromPC[buffSize] = {0};

char inputBuffer[buffSize];

double wanted_force, force_val, motor_power;

const float Kp = 1, Ki = 0.8, Kd = 4;
PID myPID(&force_val, &motor_power, &wanted_force, Kp, Ki, Kd, DIRECT);

StaticJsonDocument<JSON_OBJECT_SIZE(3)> doc;

void move_motor(int motor_power, const int THRESH = 0)
{
  motor_power = max(min(motor_power, 255), -255);

  if (motor_power < -THRESH)
  {
    digitalWrite(MOTOR_DIR_PIN, 0);
  }
  else if (motor_power > THRESH)
  {
    digitalWrite(MOTOR_DIR_PIN, 1);
  }

  analogWrite(MOTOR_PWM_PIN, abs(motor_power));
}

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

void clear_serial()
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

bool newData = false;
const byte numChars = 8;
char receivedChars[numChars];

void recvWithStartEndMarkers()
{
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // Serial.setTimeout(50);

  pinMode(13, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  enableInterrupt(ENCODER_PIN_A, encoderCount, CHANGE);
  enableInterrupt(ENCODER_PIN_B, encoderCount, CHANGE);

  clear_serial();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop()
{
  static int curr_iteration = 0;
  static int iterations = 0;
  static bool new_iteration = false;

  static const float ENCODER_KP = 1.6;
  static const float ERR_THRESH = 0.2;
  unsigned long curr_time = millis();
  static char s_doc[128];

  doc["time"] = curr_time / 1000;
  doc["iteration"] = curr_iteration;
  doc["force"] = force_val;

  serializeJson(doc, s_doc);

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
    // Serial.println(force_val);
  }

  // recvWithStartEndMarkers();

  // if (newData == true)
  // {
  //   // Serial.print("This just in ... ");
  //   // Serial.println(atof(receivedChars));
  //   force_val = (float)atof(receivedChars);
  //   newData = false;
  // }

  if (force_val == 0.0)
  {
    myPID.SetTunings(4, 0.01, 1);
  }
  else
  {
    myPID.SetTunings(Kp, Ki, Kd);
  }

  myPID.Compute();

  // calculating motor's power
  force_err = wanted_force - force_val;
  // motor_power = force_err * KP;

  // Serial.write(int(force_val));

  move_motor(motor_power);

  if (force_err < ERR_THRESH && force_err > -ERR_THRESH && !new_iteration)
  { // reached wanted force and therefore finnished the iteration.
    curr_iteration++;
    new_iteration = true;

    while (encoder_val < -10 || encoder_val > 10)
    {
      move_motor(encoder_val * ENCODER_KP);
    }
  }

  if (new_iteration && force_val == 0)
  {
    Serial.write("it");
    new_iteration = false;
  }

  if (curr_iteration == iterations)
  { // finnished all iterations

    Serial.write("it");
    Serial.write("finfinfin");

    iterations = 0;
    wanted_force = 0;
    while (true)
    {
      analogWrite(MOTOR_PWM_PIN, 0);
    }
  }
}