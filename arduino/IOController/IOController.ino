// NOTE: Commands can have a maximum length of 127 characters and are not
// sanitised, use with care.

// Commands (>> = rpi to arduino, << = arduino to rpi):
//
// >> servos <ang1> <ang2> <ang3> <ang4> <speed1> <speed2> <speed3> <speed4> (degrees, deg/s?)
// >> props <prop1> <prop2> <prop3> <prop4> (thrust in micronewton tussen 0 en 11,000)
// >> retracts <0/1> (1 = down)
// >> pollimu
// >> pollsonar
//
// << imu <pitch> <yaw> <roll> <accX> <accY> <accZ> (degrees * 1000, m/s^2 * 1000)
// << sonar <dist> (cm)
// << error (sent if command is wrong, other errors cause undefined behaviour)

#include <VarSpeedServo.h>
#include <NewPing.h>
#include <EEPROM.h>
//#include <Servo.h>
#include <Wire.h>

#include "Kalman.h"

// Configuration
const int PIN_ESC1 = 13;
const int PIN_ESC2 = 12;
const int PIN_ESC3 = 11;
const int PIN_ESC4 = 10;

const int PIN_SERVO1 = 3;
const int PIN_SERVO2 = 6;
const int PIN_SERVO3 = 9;
const int PIN_SERVO4 = 10;

const int PIN_US_TRIGGER = 52;
const int PIN_US_ECHO = 53;

const int PIN_RETRACTS = 2;

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

// IMU vars
Kalman kalmanX, kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

bool calibrated = false;
int start_t = 0;
double yawOffset = 0;

double gyroXangle, gyroYangle, gyroZangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;

uint32_t timer;
uint8_t i2cData[14];

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

    // Init I2C for IMU
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();
    start_t = millis();
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

        speeds[0] = constrain(args[0], minSpeed, maxSpeed);
        speeds[1] = constrain(args[1], minSpeed, maxSpeed);
        speeds[2] = constrain(args[2], minSpeed, maxSpeed);
        speeds[3] = constrain(args[3], minSpeed, maxSpeed);

        servo1.write(((angles[0]*-1+beta)*2), speeds[0]);
        servo2.write(((angles[1]+alpha)*2), speeds[1]);
        servo3.write(((angles[2]*-1+beta)*2), speeds[2]);
        servo4.write(((angles[3]+alpha)*2), speeds[3]);
    } else if (strcmp(command, "props") == 0) {
        sscanf(buf, "props %d %d %d %d", &args[0], &args[1], &args[2], &args[3]);

        esc1.writeMicroseconds(constrain(thrust2microseconds(args[0] / 1000.0f - 1.0f), 900, 1700));
        esc2.writeMicroseconds(constrain(thrust2microseconds(args[1] / 1000.0f - 1.0f), 900, 1700));
        esc3.writeMicroseconds(constrain(thrust2microseconds(args[2] / 1000.0f - 1.0f), 900, 1700));
        esc4.writeMicroseconds(constrain(thrust2microseconds(args[3] / 1000.0f - 1.0f), 900, 1700));
    } else if (strcmp(command, "retracts") == 0) {
        sscanf(buf, "retracts %d", &args[0]);

        retracts.write(args[0] == 1 ? 135 : 45);
    } else if (strcmp(command, "pollimu") == 0) {
        sprintf(
            response,
            "imu %d %d %d %d %d %d",
            (int) (kalAngleX * 1000),
            (int) (kalAngleY * 1000),
            (int) (gyroZangle * 1000),
            (int) (accX),
            (int) (accY),
            (int) (accZ)
        );

        Serial.println(response);
    } else if (strcmp(command, "pollsonar") == 0) {
        int uS = sonar.ping();
        int cm = uS / US_ROUNDTRIP_CM;

        sprintf(response, "sonar %d", cm);
        Serial.println(response);
    } else {
        Serial.println("error");
    }

    pollIMU();
    delay(2);
}

void pollIMU() {
    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    double gyroZrate = gyroZ / 131.0;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt - yawOffset * dt;

    if (!calibrated && millis() - start_t >= 1000) {
        yawOffset = gyroZangle;
        calibrated = true;
        gyroZangle = 0;
    }

    // Calculate the angle using a Complimentary filter
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
}