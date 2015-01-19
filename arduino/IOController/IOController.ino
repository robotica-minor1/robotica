// NOTE: Commands can have a maximum length of 127 characters and are not
// sanitised, use with care.

// Commands (>> = rpi to arduino, << = arduino to rpi):
//
// >> servos <ang1> <ang2> <ang3> <ang4> <speed1> <speed2> <speed3> <speed4> (degrees, +/- 40)
// >> props <prop1> <prop2> <prop3> <prop4> (thrust in micronewton tussen 0 en 11,000)
// >> retracts <0/1> (1 = down)
// >> pollimu
// >> pollsonar
//
// << imu <pitch> <yaw> <roll> <accX> <accY> <accZ> (degrees * 1000, m/s^2 * 1000)
// << sonar <dist> (cm)
// << ack (sent if setter command successful, pollimu+pollsonar will respond with data instead)
// << error (sent if command is wrong, other errors cause undefined behaviour)

#include <VarSpeedServo.h>
#include <NewPing.h>
#include <EEPROM.h>
//#include <Servo.h>
#include <Wire.h>

#include "Kalman.h"

// Configuration
const int PIN_ESC1 = 2;
const int PIN_ESC2 = 3;
const int PIN_ESC3 = 4;
const int PIN_ESC4 = 5;

const int PIN_SERVO1 = 13;
const int PIN_SERVO2 = 12;
const int PIN_SERVO3 = 11;
const int PIN_SERVO4 = 10;

const int PIN_US_TRIGGER = 52;
const int PIN_US_ECHO = 53;

const int PIN_RETRACTS = 48;

const int SONAR_MAX_DIST = 200;

// Globals
NewPing sonar(PIN_US_TRIGGER, PIN_US_TRIGGER, SONAR_MAX_DIST);

// Servo-like vars
VarSpeedServo retracts;
VarSpeedServo esc1, esc2, esc3, esc4;
VarSpeedServo servo1, servo2, servo3, servo4;

int angle1_mem = 0, angle2_mem = 0, angle3_mem = 0, angle4_mem = 0;

int minSpeed = 0;
int startSpeed = 30;
int maxSpeed = 127;

int minAngle = 0;
int maxAngle = 90;
int minPulse = 1000;
int maxPulse = 2000;

int alpha = 20;
int beta = 70;

int offset2 = 5;
int offset4 = -4;

// Motoren: 900 = 2000 RPM = -0.7 N, 1700 = 10600 RPM = 9.95 N
//
// Formules:
//
// rpm = microsec * 10.75 - 7675
// microsec = (rpm + 7675) / 10.75
// thrust = 0.001238372 * rpm - 3.176744
// rpm = (thrust + 3.176744) / 0.001238372
//
// Dus: 
int thrust2microseconds(float thrust) {
    float rpm = (thrust + 3.176744f) / 0.001238372f;
    return (rpm + 7675) / 10.75f;
}

void setup() {
    Serial.begin(115200);

    // Init retracts + escs
    retracts.attach(PIN_RETRACTS);

    esc1.attach(PIN_ESC1);
    esc2.attach(PIN_ESC2);
    esc3.attach(PIN_ESC3);
    esc4.attach(PIN_ESC4);

    // Init servos
    servo1.write(minPulse+((minPulse/(maxAngle))*beta), startSpeed);
    servo2.write(minPulse+((minPulse/(maxAngle))*alpha), startSpeed);
    servo3.write(minPulse+((minPulse/(maxAngle))*beta), startSpeed);
    servo4.write(minPulse+((minPulse/(maxAngle))*alpha), startSpeed);
}

void loop() {
    static char buffer[128];
    static int buf_idx = 0;

    // Accept commands
    while (Serial.available()) {
        buffer[buf_idx] = Serial.read();

        if (buffer[buf_idx] == '\n') {
            buffer[buf_idx] = 0;
            parseCommand(buffer);
            buf_idx = 0;
        } else {
            buf_idx++;
        }
    }
}

// Parse incoming commands
void parseCommand(const char* buf) {
    static char response[128];
    static char command[128];
    static int args[4];

    static int angles[4];
    static int speeds[4];

    sscanf(buf, "%s", command);

    if (strcmp(command, "servos") == 0) {
        sscanf(buf, "servos %d %d %d %d %d %d %d %d", &angles[0], &angles[1], &angles[2], &angles[3], &speeds[0], &speeds[1], &speeds[2], &speeds[3]);
        
        servo1.attach(PIN_SERVO1, 1000, 2000);
        servo2.attach(PIN_SERVO2, 1000, 2000);
        servo3.attach(PIN_SERVO3, 1000, 2000);
        servo4.attach(PIN_SERVO4, 1000, 2000);

        angles[0] = constrain(angles[0], (minAngle-(maxAngle-beta)), (maxAngle-(maxAngle-beta)));
        angles[1] = constrain(angles[1], (minAngle-alpha), (maxAngle-alpha));  
        angles[2] = constrain(angles[2], (minAngle-(maxAngle-beta)), (maxAngle-(maxAngle-beta)));
        angles[3] = constrain(angles[3], (minAngle-alpha),(maxAngle-alpha));

        speeds[0] = constrain(speeds[0], minSpeed, maxSpeed);
        speeds[1] = constrain(speeds[1], minSpeed, maxSpeed);
        speeds[2] = constrain(speeds[2], minSpeed, maxSpeed);
        speeds[3] = constrain(speeds[3], minSpeed, maxSpeed);

        servo1.write(((angles[0]*-1+beta)*2), speeds[0]);
        servo2.write(((angles[1]+alpha+offset2)*2), speeds[1]);
        servo3.write(((angles[2]*-1+beta)*2), speeds[2]);
        servo4.write(((angles[3]+alpha+offset4)*2), speeds[3]);
        
        Serial.println("ack");
    } else if (strcmp(command, "props") == 0) {
        sscanf(buf, "props %d %d %d %d", &args[0], &args[1], &args[2], &args[3]);

        esc1.writeMicroseconds(constrain(thrust2microseconds(args[0] / 1000.0f - 1.0f), 900, 1700));
        esc2.writeMicroseconds(constrain(thrust2microseconds(args[1] / 1000.0f - 1.0f), 900, 1700));
        esc3.writeMicroseconds(constrain(thrust2microseconds(args[2] / 1000.0f - 1.0f), 900, 1700));
        esc4.writeMicroseconds(constrain(thrust2microseconds(args[3] / 1000.0f - 1.0f), 900, 1700));
        
        Serial.println("ack");
    } else if (strcmp(command, "retracts") == 0) {
        sscanf(buf, "retracts %d", &args[0]);

        retracts.write(args[0] == 1 ? 135 : 45);
        
        Serial.println("ack");
    } else if (strcmp(command, "pollsonar") == 0) {
        int uS = sonar.ping();
        int cm = uS / US_ROUNDTRIP_CM;

        sprintf(response, "sonar %d", cm);
        Serial.println(response);
    } else {
        Serial.println("error");
    }

    delay(2);
}
