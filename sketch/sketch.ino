// GitHub auto-sync test
#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_VL53L1X.h>
#include <Arduino_RouterBridge.h>

// ==================== HARDWARE ====================

Adafruit_PWMServoDriver pwm(0x40);
Adafruit_VL53L1X vl53;

// ==================== SERVO SETTINGS ====================

const int SERVO_FREQ = 50;
const int MIN_PULSE_US = 600;
const int MAX_PULSE_US = 2400;

const int SERVO_COUNT = 6;

// Logical servo 1-6 -> PCA9685 CH0-CH5
const int servoChannels[SERVO_COUNT] = {
    0, 1, 2, 3, 4, 5
};

const int minAngles[SERVO_COUNT] = {
    0, 50, 0, 0, 0, 0
};

const int maxAngles[SERVO_COUNT] = {
    180, 180, 180, 180, 180, 140
};

// Milliseconds per 1-degree movement step
const unsigned long STEP_INTERVAL_MS[SERVO_COUNT] = {
    20, 25, 20, 20, 20, 80
};

int currentAngles[SERVO_COUNT] = {
    90, 90, 90, 90, 90, 90
};

int targetAngles[SERVO_COUNT] = {
    90, 90, 90, 90, 90, 90
};

bool initialized[SERVO_COUNT] = {
    false, false, false,
    false, false, false
};

unsigned long lastStep[SERVO_COUNT] = {
    0, 0, 0, 0, 0, 0
};

// ==================== SENSOR STATUS ====================

int lastDistance = -1;
bool sensorReady = false;
bool pwmReady = false;

int get_distance() {
    return lastDistance;
}

// ==================== SERVO FUNCTIONS ====================

int angleToPWM(int angle) {
    angle = constrain(angle, 0, 180);

    long pulseUs = map(
        angle,
        0,
        180,
        MIN_PULSE_US,
        MAX_PULSE_US
    );

    return (int)(pulseUs * 4096L / 20000L);
}

void writeServo(int index, int angle) {
    if (!pwmReady) {
        return;
    }

    pwm.setPWM(
        servoChannels[index],
        0,
        angleToPWM(angle)
    );
}

int set_servo(int servo, int angle) {
    if (servo < 1 || servo > SERVO_COUNT) {
        return -1;
    }

    if (!pwmReady) {
        return -3;
    }

    int index = servo - 1;

    angle = constrain(
        angle,
        minAngles[index],
        maxAngles[index]
    );

    targetAngles[index] = angle;

    // First command sets the initial servo position.
    // This may cause immediate physical movement.
    if (!initialized[index]) {
        currentAngles[index] = angle;
        initialized[index] = true;

        writeServo(index, angle);
        lastStep[index] = millis();
    }

    return angle;
}

int get_servo_angle(int servo) {
    if (servo < 1 || servo > SERVO_COUNT) {
        return -1;
    }

    int index = servo - 1;

    if (!initialized[index]) {
        return -2;
    }

    return currentAngles[index];
}

int stop_arm() {
    if (!pwmReady) {
        return -3;
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        pwm.setPWM(
            servoChannels[i],
            0,
            0
        );

        targetAngles[i] = currentAngles[i];
        initialized[i] = false;
    }

    return 1;
}

void updateServos() {
    if (!pwmReady) {
        return;
    }

    unsigned long now = millis();

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (!initialized[i]) {
            continue;
        }

        if (currentAngles[i] == targetAngles[i]) {
            continue;
        }

        if (
            now - lastStep[i] < STEP_INTERVAL_MS[i]
        ) {
            continue;
        }

        lastStep[i] = now;

        if (currentAngles[i] < targetAngles[i]) {
            currentAngles[i]++;
        }
        else {
            currentAngles[i]--;
        }

        writeServo(i, currentAngles[i]);
    }
}

// ==================== SERIAL COMMANDS ====================

char serialBuffer[64];
int serialLength = 0;

void processSerialCommand(const char* command) {
    int servo;
    int angle;

    if (strcmp(command, "PING") == 0) {
        Serial.println("PONG");
    }

    else if (strcmp(command, "STATUS") == 0) {
        Serial.print("STATUS:PWM=");
        Serial.print(pwmReady ? 1 : 0);

        Serial.print(":SENSOR=");
        Serial.println(sensorReady ? 1 : 0);
    }

    else if (strcmp(command, "GET_DISTANCE") == 0) {
        Serial.print("DISTANCE:");
        Serial.println(lastDistance);
    }

    else if (
        sscanf(command, "SERVO:%d:%d", &servo, &angle) == 2
    ) {
        int result = set_servo(servo, angle);

        Serial.print("SERVO_RESULT:");
        Serial.print(servo);
        Serial.print(":");
        Serial.println(result);
    }

    else if (strcmp(command, "STOP") == 0) {
        int result = stop_arm();

        Serial.print("STOP_RESULT:");
        Serial.println(result);
    }

    else if (
        sscanf(command, "GET_SERVO:%d", &servo) == 1
    ) {
        Serial.print("SERVO_ANGLE:");
        Serial.print(servo);
        Serial.print(":");
        Serial.println(get_servo_angle(servo));
    }

    else {
        Serial.println("ERROR:UNKNOWN_COMMAND");
    }
}

void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n') {
            serialBuffer[serialLength] = '\0';

            if (serialLength > 0) {
                processSerialCommand(serialBuffer);
            }

            serialLength = 0;
        }

        else if (c != '\r') {
            if (
                serialLength <
                (int)sizeof(serialBuffer) - 1
            ) {
                serialBuffer[serialLength++] = c;
            }
            else {
                serialLength = 0;
                Serial.println("ERROR:COMMAND_TOO_LONG");
            }
        }
    }
}

// ==================== DISTANCE SENSOR ====================

void updateDistance() {
    if (!sensorReady) {
        return;
    }

    if (!vl53.dataReady()) {
        return;
    }

    int distance = vl53.distance();

    if (distance >= 0) {
        lastDistance = distance;

        Serial.print("Distance: ");
        Serial.print(lastDistance);
        Serial.println(" mm");
    }
    else {
        lastDistance = -1;
        Serial.println("Invalid measurement");
    }

    vl53.clearInterrupt();
}

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(1000);

    Serial.println("Starting Shmolik...");

    // Initialize PCA9685
    Serial.println("Initializing PCA9685...");

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);

    pwmReady = true;

    delay(500);

    // Disable outputs on startup
    stop_arm();

    Serial.println("PCA9685 initialized");

    // Initialize VL53L1X
    Serial.println("Initializing VL53L1X...");

    if (!vl53.begin(0x29, &Wire)) {
        Serial.println(
            "ERROR: VL53L1X not found!"
        );
    }
    else if (!vl53.startRanging()) {
        Serial.println(
            "ERROR: Cannot start ranging!"
        );
    }
    else {
        vl53.setTimingBudget(50);
        sensorReady = true;

        Serial.println("VL53L1X initialized");
    }

    // RouterBridge for Arduino App Lab
    Serial.println("Initializing RouterBridge...");

    Bridge.begin();

    Bridge.provide("set_servo", set_servo);
    Bridge.provide("get_servo_angle", get_servo_angle);
    Bridge.provide("stop_arm", stop_arm);
    Bridge.provide("get_distance", get_distance);

    Serial.println("Shmolik ready!");
}

// ==================== MAIN LOOP ====================

void loop() {
    readSerialCommands();
    updateDistance();
    updateServos();

    delay(10);
}
