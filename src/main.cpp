#include <Arduino.h>
#include <EnableInterrupt.h>
#include <PID_v1.h>

volatile long encoder_val = 0;

const int MOTOR_PWM_PIN = 10;
const int MOTOR_DIR_PIN = 12;
const int ENCODER_PIN_A = 3;
const int ENCODER_PIN_B = 5;

const int DELTA_FORCE_LEN = 5;
float last_delta_forces[DELTA_FORCE_LEN];

int iterations = 0;
bool dir = 1; // 1 - push, 0 - pull

enum State
{
    PUSH = 1,
    PULL = 2,
    N
};

State state = State::N;

const byte buffSize = 40;
char messageFromPC[buffSize] = {0};

double wanted_force, force_val, motor_power;
int ticks = 0;

const float Kp = 7.3, Ki = 0, Kd = 0.2;
PID myPID(&force_val, &motor_power, &wanted_force, Kp, Ki, Kd, DIRECT);

void move_motor(int motor_pow, const bool backwards)
{
    motor_pow = max(min(motor_pow, 255), -255);

    if (backwards)
    {
        digitalWrite(MOTOR_DIR_PIN, !dir);
    }
    else
    {
        digitalWrite(MOTOR_DIR_PIN, dir);
    }

    analogWrite(MOTOR_PWM_PIN, abs(motor_pow));
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
const byte numChars = 16;
char receivedChars[numChars];

double recvWithStartEndMarkers()
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

                return atof(receivedChars);
            }
        }
        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

void get_init_data()
{
    while (wanted_force == 0 || iterations == 0 || state == State::N || ticks == 0)
    {
        if (iterations == 0)
        {
            iterations = recvWithStartEndMarkers();
            newData = false;
        }
        else if (wanted_force == 0)
        {
            wanted_force = recvWithStartEndMarkers();
            newData = false;
        }
        else if (state == State::N)
        {
            int state_tmp = recvWithStartEndMarkers();

            if (state_tmp == 1)
            {
                state = State::PUSH;
            }
            else if (state_tmp == 2)
            {
                state = State::PULL;
            }
            newData = false;
        }
        else if (ticks == 0)
        {
            ticks = recvWithStartEndMarkers();
        }

        // Serial.print(iterations);
        // Serial.print("---");
        // Serial.print((int)wanted_force);
        // Serial.print("---");
        // Serial.print(state);
        // Serial.print("---");
        // Serial.println(ticks);
    }

    Serial.write(iterations);
    Serial.write("---");
    Serial.write((int)wanted_force);
    Serial.write("---");
    Serial.write(state);
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

    // clear_serial();

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTime(20);

    get_init_data();

    dir = state == State::PUSH; // determine the direction base on user input

    for (int i = 0; i < DELTA_FORCE_LEN; i++)
    {
        last_delta_forces[i] = 0;
    }
}

void loop()
{
    static int curr_iteration = 0;
    static bool new_iteration = false;
    static float force_err = 0;

    static const float ENCODER_KP = 1.6;
    static const float ERR_THRESH = 0.2;
    static const float DT = 0.4;
    static const int FIRST_MOVE_SPEED = 30;
    static float last_force_val = 0;
    static float avg_delta_force = 0;
    static float force_val_pred = 0;
    static int touch_encoder_val = 0;

    // while (true)
    // {
    //   Serial.println(encoder_val);
    // }

    if (Serial.available() > 0)
    {
        force_val = abs(Serial.parseFloat());

        avg_delta_force = 0;

        // predicition of the real force value.
        for (int i = 1; i < DELTA_FORCE_LEN; i++)
        {
            last_delta_forces[i - 1] = last_delta_forces[i];
        }

        last_delta_forces[DELTA_FORCE_LEN - 1] = force_val - last_force_val;

        for (int i = 0; i < DELTA_FORCE_LEN; i++)
        {
            avg_delta_force += last_delta_forces[i];
        }

        avg_delta_force /= DELTA_FORCE_LEN;
        force_val_pred = force_val + DT * avg_delta_force;

        force_val = force_val_pred;

        // Serial.write((int)force_val); // for debug
        // Serial.println(force_val);
    }

    // force_val = recvWithStartEndMarkers(); // check it for extra precision

    // if (newData == true)
    // {
    //     // Serial.print("This just in ... ");
    //     // Serial.println(force_val);
    //     newData = false;
    // }

    // calculating motor's power
    force_err = wanted_force - force_val;
    myPID.Compute();

    // Serial.write(int(force_val));

    if (force_val == 0.0)
    {
        if (curr_iteration > 0)
        { // now that we know the encoder value where force is applied, we can target to that value
            int motor_free_speed = max(ENCODER_KP * abs(touch_encoder_val - encoder_val), motor_power);
            move_motor(motor_free_speed, false);
        }
        else
        { // first run is slow
            move_motor(max(2 * wanted_force, FIRST_MOVE_SPEED), false);
        }
    }
    else
    {
        if (curr_iteration == 0 && force_val >= 3)
        {
            touch_encoder_val = encoder_val;
        }

        move_motor(motor_power, force_err < ERR_THRESH);
    }

    if (force_err < ERR_THRESH)
    { // reached wanted force and therefore finnished the iteration.
        if (!new_iteration)
        {
            curr_iteration++;
            new_iteration = true;
        }

        int encoder_stop_val = (abs(touch_encoder_val) - ticks) * (abs(touch_encoder_val) / touch_encoder_val);

        while (abs(encoder_val) < abs(encoder_stop_val) - 40 || abs(encoder_val) > abs(encoder_stop_val) + 40)
        {
            move_motor((abs(encoder_val) - encoder_stop_val) * ENCODER_KP * 2, true);
        }
    }

    if (new_iteration && (encoder_val < touch_encoder_val || force_val < 0.1 * wanted_force))
    {
        Serial.write("i---------------------------");
        new_iteration = false;
        myPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
        myPID.SetOutputLimits(-1.0, 0.0); // Forces maximum down to 0.0
        myPID.SetOutputLimits(0, 255);    // Set the limits back to normal
    }

    if (curr_iteration == iterations)
    { // finnished all iterations

        Serial.write("i");
        Serial.write("f");

        iterations = 0;
        wanted_force = 0;
        while (true)
        {
            analogWrite(MOTOR_PWM_PIN, 0);
        }
    }
    last_force_val = force_val;
}