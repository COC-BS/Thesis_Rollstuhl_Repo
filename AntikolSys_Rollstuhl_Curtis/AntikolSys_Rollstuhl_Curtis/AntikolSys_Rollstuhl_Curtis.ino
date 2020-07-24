

#include <RPLidar/RPLidar.h>
#include <mcp2515/mcp2515_defs.h>
#include <mcp2515/mcp2515.h>
#include <Canbus/Canbus.h>
#include <PID_v1/PID_v1.h>
#include <Arduino.h>
#include <math.h>

RPLidar lidar;

bool showPoints = false;

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

//Drehwinkel auf Ausgangslage
float phi;
//Drehwinkel auf Orientierungspunkt
float phi2;
float dist;

int status = -1;

const byte interruptPin = 2;

#define RPLIDAR_MOTOR 3 //PWM Pin für Lidar Motorengeschwindigkeit
#define CANEn 22  // Externer Pin für CAN Shield Enable, Slave Select

//PID Variabeln Drehen
double turnedAngle = 0; //Winkel der durch Encoder ermittelt wurde
double Setpoint; //Soll-Winkel
double Input; //Differenz zwischen soll und ist winkel
double Output; //IST-Winkel, für Motorenansteuerung nutzen
//PID-Parameter
double Kp=30, Ki=10, Kd=10;
//PID Instanz erstellen
PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//PID Variabeln Distanz
double distSetpoint;
double KpD=50, KiD=1, KdD=0;
double droveDist = 0;
double distInput;
double distOutput;
PID distPID(&distInput, &distOutput, &distSetpoint, KpD, KiD, KdD, DIRECT);

int currMillis;
int oldMillis = -1;

void resetSystem (int nextStatus) {
    points[90] = {};
    edgePoints[50] = {};
    doorPoints[6] = {};
    edgeIndex = 0;

    status = nextStatus;
}

void readCAN (tCAN message) {

    int joystickDirection;
    int joystickSpeed;

    if (mcp2515_get_message(&message))
    {
        //Motorenencoder auslesen
        if(message.id == 0x7FF) //Filter ID
        {
            //Get left and right Motorspeed (RPM)
            float currSpeedLeft = (message.data[4] + message.data[5] * 256) / 21.33;
            float currSpeedRight = (message.data[6] + message.data[7] * 256) / 21.33;
            //Serial.println("Encoder:   Left: " + String(currSpeedLeft) + ";  Right: " + String(currSpeedRight));

            currMillis = millis();

            //Winkel aus Encoderdaten, turnedAngle berechnen
            float vl = (2*PI*currSpeedLeft) / 60 * 0.1725;
            float vr = (2*PI*currSpeedRight) / 60 * 0.1725;

            //Serial.println("vl: " + String(vl) + ";  vr: " + String(vr));
            //Serial.println("oldMillis: " + String(oldMillis) + "  currMillis: " +  String(currMillis));

            if (oldMillis != -1) {
                //Winkel in Radiant, Zeitschritt: Vorherige Zeit minus jetzige Zeit
                float angleRad = (vr-vl) / (2 * 0.26) * (currMillis - oldMillis) / 1000;
                //Winkel in Grad
                turnedAngle += angleRad * 180 / PI;

                droveDist += (vr + vl) / 2 *(currMillis - oldMillis) / 1000;
            }

            oldMillis = millis();
            //Serial.println("Gedrehter Winkel " + String(turnedAngle));
        }
    }
}

void writeCAN (tCAN message, int forwardSpeed, int turnRate) {
    message.id = 0x03E; //formatted in HEX
    message.header.rtr = 0;
    message.header.length = 8; //formatted in DEC
    message.data[0] = lowByte(forwardSpeed); //Forward Speed in %
    message.data[1] = highByte(forwardSpeed); //Forward Speed in %
    message.data[2] = lowByte(turnRate); //Turn Rdnf searchate in %
    message.data[3] = highByte(turnRate); //Turn Rate in %

    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&message);

    delay(10);
}

void motorCommandApproach (float distance, int nextStatus) {
    tCAN message;
    //Read Encoder Motor and calculate turned angle
    readCAN(message);
    distSetpoint = (distance)/100;
    distInput = droveDist;
    distPID.Compute();

    Setpoint = 0;
    Input = turnedAngle;
    motorPID.Compute();

   //Serial.println("Dist Setpoint: " + String(distSetpoint) + "  Output PID: " + String(distOutput) + "  Drove Dist: " + String(droveDist));
    writeCAN(message,distOutput,(Output*-1)); //Gibt Motorensteuerung anhand Output PID

    float distAbs = (distSetpoint - droveDist);

    if (distAbs < 0.05 && distAbs > -0.05) {
        Serial.println("Drove forward!      Drove Dist: " + String(droveDist));
        Serial.println("#==============================================");
        turnedAngle = 0;
        droveDist = 0;

        status = nextStatus;

    }
}

void motorCommandRotation(float phi, int nextStatus) {
    tCAN message;
    //Read Encoder Motor and calculate turned angle
    readCAN(message);
    Setpoint = phi;
    Input = turnedAngle;
    motorPID.Compute();

    //Serial.println("Output PID: " + String(Output) + "  Turned Angle: " + String(turnedAngle));
    writeCAN(message,0,(Output*-1)); //Gibt Motorensteuerung anhand Output PID

    float angleAbs = (abs(phi) - abs(turnedAngle));
    if (angleAbs < 0.1 && angleAbs > -0.1) {
        Serial.println("Turned on Point!      Turned Angle: " + String(turnedAngle));
        Serial.println("#==============================================");
        turnedAngle = 0;

        status = nextStatus;
    }
}

void calcDriveAngle () {
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
    float betrag = sqrt(pow(d.x,2) + pow(d.y,2));

    phi = 90 - (acos((s.x*d.x+s.y*d.y)/(radius * sqrt(pow(d.x,2) + pow(d.y,2))))*180/PI);


    Serial.println("#============= Drive Control Points ============");
    Serial.print("Schnittpunkt  s.x: ");
    Serial.print(s.x);
    Serial.print("  s.y: ");
    Serial.println(s.y);

    Serial.print("Mitte Tuer  d.x: ");
    Serial.print(d.x);
    Serial.print("  d.y: ");
    Serial.println(d.y);

    Serial.print("Radius: ");
    Serial.println(radius);

    Serial.print("Winkel Phi: ");
    Serial.println(phi);
    Serial.println("#==============================================");

    status = 2;
}

void wheelchairGeomtryCorrection() {
    //Korrektur Sensorversatz zum Mittelpunkt des Rollstuhls
    doorPoints[0].x -= 37;
    doorPoints[0].y += 26;
    doorPoints[1].x -= 37;
    doorPoints[1].y += 26;

    doorPoints[0].x *= (-1);
    doorPoints[0].y *= (-1);
    doorPoints[1].x *= (-1);
    doorPoints[1].y *= (-1);



    float angleRad;
    doorPoints[0].dist = sqrtf(pow( doorPoints[0].x,2)+pow( doorPoints[0].y,2));
    angleRad = atanf(doorPoints[0].y/doorPoints[0].x);
    doorPoints[0].angle = angleRad * 180 / PI;


    doorPoints[1].dist = sqrtf(pow( doorPoints[1].x,2)+pow( doorPoints[1].y,2));
    angleRad = atanf(doorPoints[1].y/doorPoints[1].x);
    doorPoints[1].angle = angleRad * 180 / PI;

    Serial.println("#======= CorrectedDoor Points ========");

    Serial.print("x: ");
    Serial.print(doorPoints[0].x);
    Serial.print("  y: ");
    Serial.print(doorPoints[0].y);
    Serial.print("  dist: ");
    Serial.print(doorPoints[0].dist);
    Serial.print("  angle: ");
    Serial.println(doorPoints[0].angle);

    Serial.print("x: ");
    Serial.print(doorPoints[1].x);
    Serial.print("  y: ");
    Serial.print(doorPoints[1].y);
    Serial.print("  dist: ");
    Serial.print(doorPoints[1].dist);
    Serial.print("  angle: ");
    Serial.println(doorPoints[1].angle);
}

void driveCommandDirect (int nextStatus) {

    wheelchairGeomtryCorrection();

    Points middleDoor;
    Points orientationPoint;

    middleDoor.dist = (doorPoints[0].dist + doorPoints[1].dist) / 2;
    middleDoor.angle = (doorPoints[0].angle + doorPoints[1].angle) / 2;
    middleDoor.x = (doorPoints[0].x + doorPoints[1].x) / 2;
    middleDoor.y = (doorPoints[0].y + doorPoints[1].y) / 2;

    phi = middleDoor.angle;
    dist = middleDoor.dist;

    Serial.println("#============= Middle Door ============");
    Serial.print("x: ");
    Serial.print(middleDoor.x);
    Serial.print("  y: ");
    Serial.println(middleDoor.y);
    Serial.print("Winkel Phi: ");
    Serial.println(phi);
    Serial.print("Distanz: ");
    Serial.println(middleDoor.dist);

    if (status == 1) {

    Vector w, m;
    w.x = (doorPoints[1].x - doorPoints[0].x);
    w.y = (doorPoints[1].y - doorPoints[0].y);

    //Distanz in x-Richtung zum Türmittelpunkt
    //Vorlagerung des ORientierungspunktes
    m.x = 90;
    m.y = (-m.x*w.x) / w.y;

    orientationPoint.x = middleDoor.x - m.x;
    orientationPoint.y = middleDoor.y - m.y;

    float angleRad;
    orientationPoint.dist = sqrtf(pow(orientationPoint.x,2)+pow(orientationPoint.y,2));
    angleRad = atanf(orientationPoint.y/orientationPoint.x);
    orientationPoint.angle = angleRad * 180 / PI;

    phi = orientationPoint.angle;
    dist = orientationPoint.dist;

    Serial.println("#============= Orientation Point ============");
    Serial.print("x: ");
    Serial.print(orientationPoint.x);
    Serial.print("  y: ");
    Serial.println(orientationPoint.y);
    Serial.print("Winkel Phi: ");
    Serial.println(phi);
    Serial.print("Distanz: ");
    Serial.println(dist);

    //Berechnung Drehwinkel auf Orientierungspunkt
    phi2 = acos((m.x*orientationPoint.x+m.y*orientationPoint.y)/
            (sqrt(pow(orientationPoint.x,2) + pow( orientationPoint.y,2)) * sqrt(pow(m.x,2) + pow(m.y,2))))*180/PI;

    if (phi < 0 && phi2 < 0 || phi > 0 && phi2 > 0) phi2 *= (-1);

    Serial.print("Winkel Phi2: ");
    Serial.println(phi2);

    }

    status = nextStatus;
}

void detectDoor (int edgeThreshold, int nextStatus) {
    doorIndex = 0;
    double mask [] = {-1, 2, -1};
    for (int i = 1; i < 90; ++i) {
        int edgeValue = points[i-1].dist * mask[0] + points[i].dist * mask[1] +
                        points[i+1].dist * mask[2];
        if (edgeValue < edgeThreshold && points[i].quality > 10) {
            edgePoints[edgeIndex] = points[i];
            edgeIndex++;
        }
    }

    if (showPoints) {
        Serial.println("#========== Edge Points ===========");
        for (int j = 0; j < edgeIndex; ++j) {
            Serial.print(edgePoints[j].angle);
            Serial.print(" ");
            Serial.print(edgePoints[j].dist);
            Serial.print(" ");
            Serial.println(edgePoints[j].quality);
        }
    }



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

        //if (showPoints) {
            Serial.print("x: ");
            Serial.print(doorPoints[j].x);
            Serial.print("  y: ");
            Serial.print(doorPoints[j].y);
            Serial.print("  dist: ");
            Serial.print(doorPoints[j].dist);
            Serial.print("  angle: ");
            Serial.println(doorPoints[j].angle);
        //}

        }

    Serial.print("DoorIndex:  ");
    Serial.println(String(doorIndex));

    if (doorIndex == 2)
        status = nextStatus;
    else if (doorIndex == 0 || doorIndex > 2)
        status = 15;
}

void scanLidar (int nextStatus) {
    for (int i = 0; i < 1000; ++i) {
        if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance; //distance value in cm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

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

    for (int i = 0; i < 90; ++i) {
        if (points[i].dist == 0) {
            points[i].angle = 135+i;
            points[i].dist = 5000;
        }

    }

    //Print points-Buffer to the Serial
    if (showPoints) {
        for (int i = 0; i < 90; ++i) {
            Serial.print(points[i].angle);
            Serial.print(" ");
            Serial.print(points[i].dist);
            Serial.print(" ");
            Serial.println(points[i].quality);
        }
    }


    //Detect Door
    analogWrite(RPLIDAR_MOTOR, 0);
    detectDoor(edgeThreshhold, nextStatus);

}

void btChange() {
    if (status == -1)
        status = 0;
    else
        status = -1;
}

void setup() {
    // bind the RPLIDAR driver to the arduino hardware serial
    lidar.begin(Serial3);
    Serial.begin(115200);

    // set pin modes
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(CANEn,OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(interruptPin), btChange, RISING);

    digitalWrite(CANEn,HIGH);
    delay(10);
    Serial.println("CAN Write - Testing transmission of CAN Bus messages");
    delay(1000);
    if(Canbus.init(CANSPEED_250)) { //Initialise MCP2515 CAN controller at the specified speed
        Serial.println("CAN Init ok");
    }
    else
        Serial.println("Can't init CAN");
    delay(1000);


    //PID-Initialisierung
    motorPID.SetMode(AUTOMATIC); //Turn PID on
    motorPID.SetTunings(Kp,Ki,Kd); //Adjust PID values
    //motorPID.SetOutputLimits(-40,40);

    distPID.SetMode(AUTOMATIC);
    distPID.SetTunings(KpD,KiD,KdD);
    distPID.SetOutputLimits(-30,30);
}


void loop() {
    //Scan for Points with the LIDAR and search a Door

    //Türdetektion als einzelner Case

    switch (status) {
        case 0:
            Serial.println("LIDAR-Scan");
            scanLidar(1);
            break;
        case 1:
            //Serial.println("Calculate Angle");
            //calcDriveAngle();
            driveCommandDirect(2);
            break;
        case 2:
            //Serial.println("Motor Commands");
            motorCommandRotation(phi,3);
            break;
        case 3:
            motorCommandApproach(dist,4);
            break;
        case 4:
            motorCommandRotation(phi2,5);
            break;
        case 5:
            motorCommandApproach(dist+40,16);
            break;
        case 15:
            Serial.println("Error, Reset System");
            resetSystem(-1);
            break;
        case 16:
            Serial.println("Door passed, Reset System");
            resetSystem(-1);
            break;
        default:
            break;
    }


    //Versatz des Sensors vom Mittelpunkt, Variable Sensor Offset!


}