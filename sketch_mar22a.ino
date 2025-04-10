#include <Wire.h>
#include <MPU9250_asukiaaa.h>

#define IR_SENSOR 35  
#define MOTOR_A1 15  
#define MOTOR_A2 2   
#define MOTOR_B1 13  
#define MOTOR_B2 4   
#define ENA 12       
#define ENB 14       

MPU9250_asukiaaa imu;
HardwareSerial BTSerial(2); // Using Serial2 for Bluetooth

volatile int pulseCount = 0;  
const int targetPulses = 12;  
volatile bool pulseDetected = false;
bool countEnabled = true;
unsigned long lastPulseTime = 0;
const int pulseDebounceTime = 50; 

const float targetAngle = 90.0; 
const int speedTurn = 225;      

int destination = 0;  // Destination choice (1, 2, or 3)

// ** IR Sensor Interrupt **
void IRAM_ATTR countPulse() {
    unsigned long currentTime = millis();
    if ((currentTime - lastPulseTime) > pulseDebounceTime) {
        if (countEnabled) {
            pulseCount++;
            pulseDetected = true;
            lastPulseTime = currentTime;
        }
    }
}

// ** Motor Control Functions **
void moveForward() {
    pulseCount = 0;  // Reset pulse count for forward movement
    BTSerial.println("Motors Moving Forward...");
    ledcWrite(0, 130);
    ledcWrite(1, 130);
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void moveBackward() {
    pulseCount = 0;  // Reset pulse count for backward movement
    BTSerial.println("Motors Moving Backward...");
    ledcWrite(0, 130);
    ledcWrite(1, 130);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
}

void turnRight() {
    BTSerial.println("Turning Right...");
    ledcWrite(0, speedTurn);
    ledcWrite(1, speedTurn);
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
}

void turnLeft() {
    BTSerial.println("Turning Left...");
    ledcWrite(0, speedTurn);
    ledcWrite(1, speedTurn);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void stopMotors() {
    BTSerial.println("Stopping Motors...");
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
}

// ** Read Gyro Angle for Turning **
float getGyroAngle() {
    imu.gyroUpdate();
    float rawRate = imu.gyroZ();
    return rawRate * 1.1; 
}

// ** Turn Right with Gyro Correction **
void turnRightWithGyro() {
    BTSerial.println("Starting Right Turn...");
    float angle = 0;
    unsigned long lastTime = millis();

    turnRight();

    while (angle < targetAngle) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        lastTime = now;

        float rate = getGyroAngle();
        angle += rate * dt;

        BTSerial.print("Right Turn Angle: ");
        BTSerial.println(angle);
    }

    stopMotors();
    BTSerial.println("Right Turn Complete!");
}

// ** Turn Left with Gyro Correction **
void turnLeftWithGyro() {
    BTSerial.println("Starting Left Turn...");
    float angle = 0;
    unsigned long lastTime = millis();

    turnLeft();

    while (angle > -targetAngle) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        lastTime = now;

        float rate = getGyroAngle();
        angle += rate * dt;

        BTSerial.print("Left Turn Angle: ");
        BTSerial.println(angle);
    }

    stopMotors();
    BTSerial.println("Left Turn Complete!");
}

// ** Move a Specific Distance **
void moveDistance(int pulses, bool forward) {
    BTSerial.println("Starting movement...");
    pulseCount = 0;  // Reset pulse count
    countEnabled = true;

    if (forward) {
        moveForward();
    } else {
        moveBackward();
    }

    while (pulseCount < pulses) {
        if (pulseDetected) {
            BTSerial.print("Pulse Count: ");
            BTSerial.println(pulseCount);
            pulseDetected = false;
        }
    }

    stopMotors();
    BTSerial.println("Target reached!");
}

// ** Move to Selected Destination **
void moveToDestination(int dest) {
    if (dest == 1) {
        moveDistance(targetPulses, true);
        delay(1000);
        moveDistance(targetPulses, false);
    }
    else if (dest == 2) {
        moveDistance(targetPulses, true);
        delay(1000);
        turnRightWithGyro();
        delay(1000);
        moveDistance(targetPulses, true);
        delay(1000);
        moveDistance(targetPulses, false);
        delay(1000);
        turnLeftWithGyro();
        delay(1000);
        moveDistance(targetPulses, false);
    }
    else if (dest == 3) {
        turnRightWithGyro();
        delay(1000);
        moveDistance(targetPulses, true);
        delay(1000);
        moveDistance(targetPulses, false);
        delay(1000);
        turnLeftWithGyro();
    }
}

// ** Setup Function **
void setup() {
    Serial.begin(115200);
    BTSerial.begin(9600, SERIAL_8N1, 16, 17); // RX: 16, TX: 17

    pinMode(IR_SENSOR, INPUT);
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(ENA, 0);
    ledcAttachPin(ENB, 1);

    Wire.begin(21, 22);
    imu.setWire(&Wire);
    imu.beginGyro();
    BTSerial.println("MPU9250 Gyro Ready!");

    attachInterrupt(digitalPinToInterrupt(IR_SENSOR), countPulse, FALLING);
    
    BTSerial.println("System Ready!");
}

// ** Main Loop **
void loop() {
    BTSerial.println("Enter destination (1, 2, or 3):");
    
    while (!BTSerial.available());  // Wait for input
    destination = BTSerial.parseInt();
    BTSerial.print("Selected Destination: ");
    BTSerial.println(destination);

    if (destination >= 1 && destination <= 3) {
        moveToDestination(destination);
    } else {
        BTSerial.println("Invalid destination! Choose 1, 2, or 3.");
    }

    delay(2000); // Wait before accepting next command
}