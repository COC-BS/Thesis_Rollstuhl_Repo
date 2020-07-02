
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar/RPLidar.h>
#include <Arduino.h>
#include <math.h>

// You need to create an driver instance
RPLidar lidar;

struct Vector {
    float x;
    float y;
};

struct Points {
    int angle;
    int dist;
    int quality;
    float x;
    float y;
};
Points points[90];
Points edgePoints[50];
Points doorPoints[6];
int edgeIndex = 0;
int doorIndex = 0;

float angleReadMin = 135;
float angleReadMax = 225;

int sensorOffsetX = 40;
int sensorOffsetY = 20;

int edgeThreshhold = -50;

boolean scanFlag = true;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

void motorCommands() {




}


void driveControl () {
    //Joystick Addresse: 0x081; Byte 0 = Direction; Byte 1 = Speed

    Points middleDoor;
    middleDoor.dist = (doorPoints[0].dist + doorPoints[1].dist) / 2;
    middleDoor.angle = (doorPoints[0].angle + doorPoints[1].angle) / 2;
    middleDoor.x = (doorPoints[0].y + doorPoints[1].x) / 2;
    middleDoor.y = (doorPoints[0].y + doorPoints[1].y) / 2;

    Vector d, w, m, s;
    d.x = middleDoor.x - sensorOffsetX;
    d.y = middleDoor.y - sensorOffsetY;
    w.x = (doorPoints[0].x - doorPoints[1].x) - sensorOffsetX;
    w.y = (doorPoints[0].y - doorPoints[1].y) - sensorOffsetY;

    m.x = 1;
    m.y = (-1*d.x) / d.y;

    float b = (m.y * d.x - m.x * d.y) / (2*(m.x*w.y-m.y*w.x));
    float k = (w.y * d.x -w.x * d.y) / (2*(m.x*w.y-m.y*w.x));

    s.x = d.x + b * w.x;
    s.y = d.y + b * w.y;

    float radius = sqrt(pow(s.x,2) + pow(s.y,2));


    Serial.println("#============= Drive Control Points ============");
    Serial.print("Mitte Tuer  d.x: ");
    Serial.print(d.x);
    Serial.print("  d.y: ");
    Serial.println(d.y);

    Serial.print("Schnittpunkt  s.x: ");
    Serial.print(s.x);
    Serial.print("  s.y: ");
    Serial.println(s.y);

    Serial.print("Radius: ");
    Serial.println(radius);

    delay(10000);
}
               


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

            float b = edgePoints[j].dist / 10;
            float c = edgePoints[i].dist / 10;
            float dist = sqrtf(pow(b, 2) + pow(c, 2) - 2 * b * c * angleRad);
            dist = dist * 10;

            if (dist > 60 && dist < 105) {
                door = true;
                int distDoor = (edgePoints[j].dist + edgePoints[i].dist) / 2 + 100;

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

    for (int i = 0; i < doorIndex; ++i) {
        doorPoints[i].x = doorPoints[i].dist * cos(doorPoints[i].angle * PI / 180);
        doorPoints[i].y = doorPoints[i].dist * sin(doorPoints[i].angle * PI / 180);
    }

    Serial.println("#========== Door Points ===========");

    //Print Door Points
    for (int j = 0; j < doorIndex; ++j) {
        /*
        Serial.print(doorPoints[j].angle);
        Serial.print(" ");
        Serial.print(doorPoints[j].dist);
        Serial.print(" ");
        Serial.print(doorPoints[j].quality);
        */
        Serial.print("  x: ");
        Serial.print(doorPoints[j].x);
        Serial.print("  y: ");
        Serial.println(doorPoints[j].y);
    }

    if (door) {
        driveControl();
    }
}

void scanLidar () {
    if (millis() < 3000) {

        if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance; //distance value in cm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

            if (distance <= 0)
                distance = 7000;

            //If point is in front of the LIDAR (135 - 225)
            if (angle <= angleReadMax && angle >= angleReadMin) {
                int rndAngle = static_cast<int>(angle);
                points[rndAngle-135].angle = rndAngle;
                points[rndAngle-135].dist = (distance/10);
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

    //Print points-Buffer to the Serial
    else if (scanFlag == true) {
        /*
        for (int i = 0; i < 90; ++i) {
            Serial.print(points[i].angle);
            Serial.print(" ");
            Serial.print(points[i].dist);
            Serial.print(" ");
            Serial.println(points[i].quality);
        }
         */

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
    //Scan for Points with the LIDAR and search a Door
    scanLidar();


    //State Machine
    //Flag wenn mehrere Türen gefunden werden und Error!!
    //Versatz des Sensors vom Mittelpunkt, Variable Sensor Offset!


}