#include <BotNRoll.h>

BotNRoll robot;

/* The setup function runs once when you press reset or power the board */
void setup() {
    // Set baud rate to 9600bps for printing values on the serial monitor.
    Serial.begin(9600);

    // Initialize the robot.
    robot.begin();

    // Wait for button (PB1) to be pressed.
    while (robot.readButton() != 1) {

        // Set servo position to 0.
        robot.servo1(0);

        // Get distances from the laser sensors on the front of the robot.
        uint16_t left = robot.getLidarLeftDistance();
        uint16_t front = robot.getLidarFrontDistance();
        uint16_t right = robot.getLidarRightDistance();

        // Print distance values to serial monitor.
        Serial.println("LIDAR VALUES:");
        Serial.println("Left: " + String(left) + "mm");
        Serial.println("Front: " + String(front) + "mm");
        Serial.println("Right: " + String(right) + "mm");

        // Get brightness values from the line sensor array on the bottom of the robot.
        int sensors[8];
        for (int i = 0; i < 8; i++) {
            sensors[i] = robot.readAdc(i);
        }

        // Print sensor values to serial monitor.
        Serial.println("LINE SENSOR VALUES:");
        for (int i = 0; i < 8; i++) {
            Serial.print(String(sensors[i]) + ", ");
        }
        Serial.println();

        // Print LDR value to serial monitor.
        Serial.println("LDR SENSOR VALUE:");
        Serial.print(robot.getLDRValue());
        Serial.println();

        delay(2000);
    }

}

/* The loop function runs over and over again forever */
void loop() {
    robot.servo1(180);
    delay(2000);
    robot.servo1(0);
    delay(2000);
}
    

