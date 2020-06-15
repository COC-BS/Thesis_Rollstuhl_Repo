
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar/RPLidar.h>
#include <Arduino.h>

// You need to create an driver instance
RPLidar lidar;

struct Points {
    int angle;
    int dist;
    int quality;
};
Points points[90];
float angleReadMin = 135;
float angleReadMax = 225;

boolean scanFlag = true;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

void scanLidar () {
    if (millis() < 5000) {

        if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

            //If point is in front of the LIDAR (135 - 225)
            if (angle <= angleReadMax && angle >= angleReadMin) {
                int rndAngle = static_cast<int>(angle);
                points[rndAngle-135].angle = rndAngle;
                points[rndAngle-135].dist = distance;
                points[rndAngle-135].quality = quality;
            }

        } else {
            analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

            // try to detect RPLIDAR...
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
                // detected...
                lidar.startScan();

                // start motor rotating at max allowed speed
                analogWrite(RPLIDAR_MOTOR, 255);
                delay(1000);
            }
        }
    }
        //Print points-Buffer to the Serial Monitor
    else if (scanFlag == true) {
        for (int i = 0; i < 90; ++i) {
            Serial.print(points[i].angle);
            Serial.print(" ");
            Serial.print(points[i].dist);
            Serial.print(" ");
            Serial.println(points[i].quality);
        }
        scanFlag = false;
        analogWrite(RPLIDAR_MOTOR, 0);
    }


}


void setup() {
    // bind the RPLIDAR driver to the arduino hardware serial
    lidar.begin(Serial3);
    Serial.begin(115200);

    // set pin modes
    pinMode(RPLIDAR_MOTOR, OUTPUT);
}


void loop() {
    //Scan for Points with the LIDAR
    scanLidar();


}