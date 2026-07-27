#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// =====================================================
// PCA9685
// =====================================================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// =====================================================
// SERVO CONFIG
// =====================================================
#define SERVO_FREQ 50

#define SERVOMIN 110
#define SERVOMAX 490

// =====================================================
// SERVO CHANNEL
// =====================================================
#define SERVO_BASE      0
#define SERVO_SHOULDER  1
#define SERVO_ELBOW     2
#define SERVO_GRIPPER   3

// =====================================================
// SERIAL DATA
// =====================================================
String serialData = "";

// =====================================================
// SERVO POSITION
// =====================================================
int servoAngle[4] = {90, 90, 90, 45};


// =====================================================
// ANGLE -> PWM
// =====================================================
int angleToPulse(int angle)
{
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}


// =====================================================
// SET SERVO
// =====================================================
void setServo(int channel, int angle)
{
  angle = constrain(angle, 0, 180);

  int pulse = angleToPulse(angle);

  pca.setPWM(channel, 0, pulse);
}


// =====================================================
// UPDATE ALL SERVO
// =====================================================
void updateServo()
{
  setServo(SERVO_BASE,     servoAngle[0]);
  setServo(SERVO_SHOULDER, servoAngle[1]);
  setServo(SERVO_ELBOW,    servoAngle[2]);
  setServo(SERVO_GRIPPER,  servoAngle[3]);
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  Serial.println("ESP32 ROBOT ARM START");

  // I2C ESP32
  Wire.begin(21, 22);

  // PCA9685
  pca.begin();

  pca.setPWMFreq(SERVO_FREQ);

  delay(1000);

  // posisi awal
  updateServo();

  Serial.println("READY");
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
  // ================================================
  // ADA DATA SERIAL
  // ================================================
  if (Serial.available())
  {
    serialData = Serial.readStringUntil('\n');

    serialData.trim();

    Serial.print("RX: ");
    Serial.println(serialData);

    // ================================================
    // PARSE JSON
    // ================================================
    StaticJsonDocument<200> doc;

    DeserializationError error =
      deserializeJson(doc, serialData);

    // ================================================
    // JSON ERROR
    // ================================================
    if (error)
    {
      Serial.print("JSON ERROR: ");
      Serial.println(error.c_str());
      return;
    }

    // ================================================
    // EMERGENCY STOP
    // ================================================
    if (doc.containsKey("cmd"))
    {
      const char* cmd = doc["cmd"];

      if (String(cmd) == "stop")
      {
        Serial.println("EMERGENCY STOP");

        servoAngle[0] = 90;
        servoAngle[1] = 90;
        servoAngle[2] = 90;
        servoAngle[3] = 45;

        updateServo();

        return;
      }
    }

    // ================================================
    // AMBIL ARRAY SERVO
    // ================================================
    if (doc.containsKey("s"))
    {
      JsonArray arr = doc["s"];

      if (arr.size() >= 4)
      {
        servoAngle[0] = constrain(arr[0], 0, 180);
        servoAngle[1] = constrain(arr[1], 0, 180);
        servoAngle[2] = constrain(arr[2], 0, 180);
        servoAngle[3] = constrain(arr[3], 0, 180);

        updateServo();

        // debug
        Serial.print("BASE: ");
        Serial.print(servoAngle[0]);

        Serial.print(" | SHOULDER: ");
        Serial.print(servoAngle[1]);

        Serial.print(" | ELBOW: ");
        Serial.print(servoAngle[2]);

        Serial.print(" | GRIPPER: ");
        Serial.println(servoAngle[3]);
      }
    }
  }
}