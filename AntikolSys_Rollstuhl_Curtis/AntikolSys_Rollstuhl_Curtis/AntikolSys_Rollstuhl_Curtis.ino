
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar/RPLidar.h>
#include <Arduino.h>
#include <math.h>

// You need to create an driver instance
RPLidar lidar;

struct Points {
    int angle;
    int dist;
    int quality;
};
Points points[90];
Points edgePoints[50];
Points doorPoints[6];
int edgeIndex = 0;
int doorIndex = 0;

float angleReadMin = 135;
float angleReadMax = 225;

int edgeThreshhold = -500;

boolean scanFlag = true;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

void detectDoor (int edgeThreshold) {
    double mask [] = {-1, 2, -1};
    for (int i = 1; i < 90; ++i) {
        int edgeValue = points[i-1].dist * mask[0] + points[i].dist * mask[1] +
                        points[i+1].dist * mask[2];
        if (edgeValue < edgeThreshold && points[i].quality > 10) {
            edgePoints[edgeIndex] = points[i];
            edgeIndex++;
        }
    }

    /*
    Serial.println("#========== Edge Points ===========");
    for (int j = 0; j < edgeIndex; ++j) {
            Serial.print(edgePoints[j].angle);
            Serial.print(" ");
            Serial.print(edgePoints[j].dist);
            Serial.print(" ");
            Serial.println(edgePoints[j].quality);
    }
     */


    boolean door = false;

    int compareIndex = 0;

    for (int j = 0; j < edgeIndex; ++j) {
        compareIndex++;
        for (int i = compareIndex; i < edgeIndex; ++i) {
            float angleRad = cos((edgePoints[j].angle - edgePoints[i].angle) * PI / 180);

            float b = edgePoints[j].dist / 100;
            float c = edgePoints[i].dist / 100;
            float dist = sqrtf(pow(b, 2) + pow(c, 2) - 2 * b * c * angleRad);
            dist = dist * 100;

            if (dist > 600 && dist < 1050) {
                door = true;
                int distDoor = (edgePoints[j].dist + edgePoints[i].dist) / 2 + 1000;

                for (int k = 0; k < 90; ++k) {
                    if (points[k].angle > edgePoints[j].angle && points[k].angle < edgePoints[i].angle) {
                        if (points[k].dist < distDoor)
                            door = false;
                    }
                }


                if (door) {
                    doorPoints[doorIndex] = edgePoints[j];
                    doorIndex++;
                    doorPoints[doorIndex] = edgePoints[i];
                    doorIndex++;
                    Serial.println("Door found!");
                }
            }

        }
    }

    Serial.println("#========== Door Points ===========");

    for (int j = 0; j < doorIndex; ++j) {
        Serial.print(doorPoints[j].angle);
        Serial.print(" ");
        Serial.print(doorPoints[j].dist);
        Serial.print(" ");
        Serial.println(doorPoints[j].quality);
    }

}

void scanLidar () {
    if (millis() < 3000) {

        if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

            if (distance <= 0)
                distance = 10000;

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
        //Detect Door
        detectDoor(edgeThreshhold);
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