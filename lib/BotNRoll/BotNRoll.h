#include <BnrOneA.h>
#include <VL53L0X.h>
#include <Wire.h>

class BotNRoll : public BnrOneA {
private:
    static constexpr uint16_t PIN_SS_BNRONEA = 2;
    static constexpr uint16_t PIN_XSHUT_RIGHT = 7;
    static constexpr uint16_t PIN_XSHUT_FRONT = 6;
    static constexpr uint16_t PIN_XSHUT_LEFT = 5;

    static constexpr uint16_t ADDR_LIDAR_RIGHT = 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT = 0x71;
    static constexpr uint16_t ADDR_LIDAR_LEFT= 0x72;

    static constexpr uint16_t DIST_LIDAR_MIN = 0;
    static constexpr uint16_t DIST_LIDAR_MAX = 2600;

    static constexpr uint16_t LDR_PIN = A0;

    VL53L0X deviceLidarRight;
    VL53L0X deviceLidarFront;
    VL53L0X deviceLidarLeft;

    void setupLidar();
    
public:
    void begin();

    uint16_t getLidarRightDistance();
    uint16_t getLidarFrontDistance();
    uint16_t getLidarLeftDistance();

    uint16_t getLDRValue();
};

void BotNRoll::begin() {
    spiConnect(PIN_SS_BNRONEA);
    stop();
    obstacleEmitters(OFF);
    minBat(3);

    setupLidar();
    servo1(0);
}

void BotNRoll::setupLidar() {
    Wire.begin();

    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);

    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(200);
    deviceLidarRight.setAddress(ADDR_LIDAR_RIGHT);
    deviceLidarRight.setTimeout(500);
    deviceLidarRight.init(true);

    digitalWrite(PIN_XSHUT_FRONT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(200);
    deviceLidarFront.setAddress(ADDR_LIDAR_FRONT);
    deviceLidarFront.setTimeout(500);
    deviceLidarFront.init(true);

    digitalWrite(PIN_XSHUT_LEFT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(200);
    deviceLidarLeft.setAddress(ADDR_LIDAR_LEFT);
    deviceLidarLeft.setTimeout(500);
    deviceLidarLeft.init(true);

    deviceLidarRight.startContinuous(0);
    deviceLidarFront.startContinuous(0);
    deviceLidarLeft.startContinuous(0);
}

/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] (mm)
 */
uint16_t BotNRoll::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotNRoll::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotNRoll::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the LDR sensor value.
 * @return value between [0, 1023] 
 */
uint16_t BotNRoll::getLDRValue() {
    uint16_t result = analogRead(LDR_PIN);
    return result;
 }