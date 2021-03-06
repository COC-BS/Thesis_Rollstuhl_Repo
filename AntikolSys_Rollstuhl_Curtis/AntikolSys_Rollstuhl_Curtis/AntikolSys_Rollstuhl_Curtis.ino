#include <RPLidar/RPLidar.h>
#include <mcp2515/mcp2515_defs.h>
#include <mcp2515/mcp2515.h>
#include <Canbus/Canbus.h>
#include <PID_v1/PID_v1.h>
#include <Arduino.h>
#include <math.h>

bool showPoints = false; //Anusgabe von Messpunkten im Serial Monitor

int status = -1; //Statusvariable für State-Machine

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

//Winkel der vom LIDAR gemessen werden soll
float angleReadMin = 135;
float angleReadMax = 225;

//Sesnorversatz vom Rollstuhlmittelpunkt
int sensorOffsetX = 40;
int sensorOffsetY = 20;

//Grenzwert für Kantendetektion
int edgeThreshhold = -80;

//Drehwinkel auf Ausgangslage
float phi;
//Drehwinkel auf Orientierungspunkt
float phi2;
float dist;

int joystickSpeed;
int joystickDirection;

const byte interruptPin = 2; //Button-Pin

#define RPLIDAR_MOTOR 3 //PWM Pin für Lidar Motorengeschwindigkeit
#define CANEn 22  // Externer Pin für CAN Shield Enable, Slave Select
#define LEDRED 8
#define LEDGREEN 10
#define LEDBLUE 9

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

//Zeitstempel für Encoderberechnung
unsigned long currMillis;
unsigned long oldMillis = -1;

/**
 * Setzt alle Variabeln und Messpunkte zurück
 * @param nextStatus: Status welcher als nächstes aufgerufen werden soll
 */
void resetSystem (int nextStatus) {
    for (int j = 0; j < 90; ++j) {
        points[j].dist = 0;
        points[j].angle = 0;
        points[j]. quality = 0;
        points[j].x = 0;
        points[j].y = 0;
    }

    for (int k = 0; k < 50; ++k) {
        edgePoints[k].dist = 0;
        edgePoints[k].angle = 0;
        edgePoints[k]. quality = 0;
        edgePoints[k].x = 0;
        edgePoints[k].y = 0;
    }

    for (int i = 0; i < 6; ++i) {
        doorPoints[i].dist = 0;
        doorPoints[i].angle = 0;
        doorPoints[i]. quality = 0;
        doorPoints[i].x = 0;
        doorPoints[i].y = 0;
    }
    edgeIndex = 0;
    doorIndex = 0;
    phi = 0;
    phi2 = 0;
    dist = 0;

    oldMillis = -1;

    status = nextStatus;
}
/**
 * Schaltet entsprechende LED-Kombination ein
 * @param ledcolor: Wert um Muster zu wählen
 */
void led(int ledcolor) {
    switch (ledcolor) {
        case 0: //Grün
            digitalWrite(LEDGREEN, HIGH);
            digitalWrite(LEDBLUE, LOW);
            digitalWrite(LEDRED, LOW);
            break;
        case 1: //Blau
            digitalWrite(LEDGREEN, LOW);
            digitalWrite(LEDBLUE, HIGH);
            digitalWrite(LEDRED, LOW);
            break;
        case 2: //Rot
            digitalWrite(LEDGREEN, LOW);
            digitalWrite(LEDBLUE, LOW);
            digitalWrite(LEDRED, HIGH);
            break;
        case 3: //Grün&Blau
            digitalWrite(LEDGREEN, HIGH);
            digitalWrite(LEDBLUE, HIGH);
            digitalWrite(LEDRED, LOW);
            break;
    }
}

/**
 * Motorenencoderwerte auf dem CAN-Bus auslesen
 * @param message
 */
void readCAN (tCAN message) {
    if (mcp2515_get_message(&message))
    {
        //Joystickdaten auslesen
        if(message.id == 0x081) //CAN-ID Joystickdaten
        {
            joystickDirection = message.data[0];
            joystickSpeed = message.data[1];
            //Serial.println("Joystick:   Direction: " + String(joystickDirection) + ";  Speed: " + String(joystickSpeed));
        }

        //Motorenencoder auslesen
        if(message.id == 0x7FF) //CAN-ID Motorenencoder
        {
            //Linke und Rechte Motroengeschwindigkeit (RPM), 21.33 -> Encoder-Rad Verhältnis
            float currSpeedLeft = (message.data[4] + message.data[5] * 256) / 21.33;
            float currSpeedRight = (message.data[6] + message.data[7] * 256) / 21.33;
            //Serial.println("Encoder:   Left: " + String(currSpeedLeft) + ";  Right: " + String(currSpeedRight));

            currMillis = millis();

            //Radgeschiwndigkeiten [m/s]
            float vl = (2*PI*currSpeedLeft) / 60 * 0.1725;
            float vr = (2*PI*currSpeedRight) / 60 * 0.1725;

            //Serial.println("vl: " + String(vl) + ";  vr: " + String(vr));
            //Serial.println("oldMillis: " + String(oldMillis) + "  currMillis: " +  String(currMillis));

            if (oldMillis != -1) {
                //Winkel in Radiant, Zeitschritt: Vorherige Zeit minus jetzige Zeit
                float angleRad = (vr-vl) / (2 * 0.26) * (currMillis - oldMillis) / 1000;
                //Gesamt gedrehter Winkel in Grad
                turnedAngle += angleRad * 180 / PI;
                //Gesamt zurückgelegte Distanz
                droveDist += (vr + vl) / 2 *(currMillis - oldMillis) / 1000;
            }

            oldMillis = millis();
            //Serial.println("Gedrehter Winkel " + String(turnedAngle));
        }
    }
}

/**
 * Fahrbefehle auf CAN-Bus schreiben
 * @param message
 * @param forwardSpeed, (0-100)
 * @param turnRate, (+ rechts Kurve | - links Kurve | 0 Geradeaus || 100 = L vorwärtas, R rückwärts, 50 nur ein Rad in Betrieb
 */
void writeCAN (tCAN message, int forwardSpeed, int turnRate) {
    message.id = 0x03E; //CAN-ID für Motorenansteuerung
    message.header.rtr = 0;
    message.header.length = 8;
    message.data[0] = lowByte(forwardSpeed); //Vorwärtsgeschwindigkeit in %
    message.data[1] = highByte(forwardSpeed);
    message.data[2] = lowByte(turnRate); //Drehgeschwindigkeit in %
    message.data[3] = highByte(turnRate);

    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&message);

    delay(10);
}

/**
 * Joystickeingaben kompensieren
 * @param driveForward
 * @param mCtlForwardSpeed
 * @param mCtlTurnRate
 * @param message
 */
void compensateJoystick(bool driveForward, int mCtlForwardSpeed, int mCtlTurnRate, tCAN message) {
    int forwardSpeed;
    int turnRate;

    readCAN(message);

    //Joystick Vorwärts-/Rückwärtseingaben kompensieren
    if (joystickSpeed < 120) {
        forwardSpeed = joystickSpeed * (-1);

        //Vorwärtsfahrt zulassen bei Türanfahrt
        if (driveForward)
            forwardSpeed += joystickSpeed * 0.2;
    }
    else {
        forwardSpeed = 100 - (joystickSpeed - 150);
    }

    //Joystick Links-/Rechtseingaben kompensieren
    if (joystickDirection < 120) {
        turnRate = joystickDirection * (-1);
    }
    else {
        turnRate = 100 - (joystickDirection - 150);
    }

    //Durchfahrtsabbruch mittels Joystickeingabe
    if ((joystickDirection > 50 && joystickDirection < 100) || (joystickDirection > 150 && joystickDirection < 200)) {
        status = 16;
    }


    //Serial.println("forwardSpeed:  " + String(forwardSpeed) + "   turnRate:  " + String(turnRate));
    forwardSpeed += mCtlForwardSpeed;
    turnRate += mCtlTurnRate;

    writeCAN(message, forwardSpeed, turnRate);
}

/**
 * Assistierte Vorwärtsfahrt, geregelt durch PID-Regler
 * @param distance
 * @param nextStatus
 * @param message
 */
void motorCommandApproach (float distance, int nextStatus, tCAN message) {
    //Read Encoder Motor and calculate turned angle
    readCAN(message);
    distSetpoint = (distance)/100;
    distInput = droveDist;
    distPID.Compute();

    Setpoint = 0;
    Input = turnedAngle;
    motorPID.Compute();

   //Serial.println("Dist Setpoint: " + String(distSetpoint) + "  Output PID: " + String(distOutput) + "  Drove Dist: " + String(droveDist));

    compensateJoystick(true, 0, (Output*-1), message);

    float distAbs = (distSetpoint - droveDist);

    if (distAbs < 0.1 && distAbs > -0.1) {
        Serial.println("Drove forward!      Drove Dist: " + String(droveDist));
        Serial.println("#==============================================");
        turnedAngle = 0;
        droveDist = 0;

        status = nextStatus;
    }
}

/**
 * Assistierte Drehung, geregelt durch PID-Regler
 * @param phi
 * @param nextStatus
 * @param message
 */
void motorCommandRotation(float phi, int nextStatus, tCAN message) {
    //Encoder auslesen und bereits gedrehter Winkel ermitteln
    readCAN(message);
    //PID-Regelung
    Setpoint = phi;
    Input = turnedAngle;
    motorPID.Compute();
    //Serial.println("Output PID: " + String(Output) + "  Turned Angle: " + String(turnedAngle));

    //Geregelte Drehung inkl. Joystickkompensation
    compensateJoystick(false, 0, (Output*-1),message);

    float angleAbs = (abs(phi) - abs(turnedAngle));
    if (angleAbs < 0.1 && angleAbs > -0.1) {
        Serial.println("Turned on Point!      Turned Angle: " + String(turnedAngle));
        Serial.println("#==============================================");
        turnedAngle = 0;

        status = nextStatus;
    }
}

/**
 * Fahrwegberechnung, Anfahrt im Kreisbogen gemäss P5
 */
void calcDriveAngle () {
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

    status = 2;
}

/**
 * Korrektur LIDAR-Versatz zum Rollstuhlmittelpunkt
 */
void wheelchairGeomtryCorrection() {
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

/**
 * Fahrwegberechnung mit Drehen auf der Stelle und Gerader Vorwärtsfahrt
 * @param nextStatus
 */
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

    Vector w, m;
    w.x = (doorPoints[1].x - doorPoints[0].x);
    w.y = (doorPoints[1].y - doorPoints[0].y);

    //Distanz in x-Richtung zum Türmittelpunkt
    //Vorlagerung des Orientierungspunktes
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
    /*
    for (int i = 0; i < 200; ++i) {
        if (IS_OK(lidar.waitPoint())) {
            led(1);
            float distance = lidar.getCurrentPoint().distance; //distance value in cm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement


            //If point is in front of the LIDAR (135 - 225)
            if (angle <= angleReadMax && angle >= angleReadMin) {
                int rndAngle = static_cast<int>(angle);
                points[rndAngle - 135].angle = rndAngle;
                points[rndAngle - 135].dist = (distance / 10);
                points[rndAngle - 135].quality = quality;
            }


        } else {
            analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
            // try to detect RPLIDAR...

            // detected...
            lidar.startScan();
            // start motor rotating at max allowed speed
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(3000);
        }
    }
     */

    for (int i = 0; i < 1000; ++i) {
        if (IS_OK(lidar.waitPoint())) {

            float distance = lidar.getCurrentPoint().distance; //distance value in cm unit
            float angle = lidar.getCurrentPoint().angle; //anglue value in degree
            byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

            //If point is in front of the LIDAR (135 - 225)
            if (angle <= angleReadMax && angle >= angleReadMin) {
                int rndAngle = static_cast<int>(angle);
                points[rndAngle - 135].angle = rndAngle;
                points[rndAngle - 135].dist = distance;
                points[rndAngle - 135].quality = quality;
            }
        } else {
            analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
            delay(1000);
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(1000);
        }
    }

    for (int i = 0; i < 90; ++i) {
        if (points[i].dist == 0) {
            points[i].angle = 135+i;
            points[i].dist = 5000;
        }
        else {
            points[i].dist /= 10;
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
}

void setup() {
    // bind the RPLIDAR driver to the arduino hardware serial
    lidar.begin(Serial3);
    Serial.begin(115200);

    // set pin modes
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(CANEn,OUTPUT);
    pinMode(LEDRED, OUTPUT);
    pinMode(LEDBLUE, OUTPUT);
    pinMode(LEDGREEN, OUTPUT);
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

    tCAN message;

    switch (status) {
        case 0:
            Serial.println("LIDAR-Scan");
            //compensateJoystick(false,0,0,message);
            led(3);
            scanLidar(1);
            break;
        case 1:
            //calcDriveAngle();
            compensateJoystick(false,0,0,message);
            led(3);
            driveCommandDirect(2);
            break;
        case 2:
            led(3);
            motorCommandRotation(phi,3, message);
            break;
        case 3:
            led(1);
            motorCommandApproach(dist-10,4, message);
            break;
        case 4:
            led(3);
            motorCommandRotation(phi2,5, message);
            break;
        case 5:
            led(1);
            motorCommandApproach(dist+40,16, message);
            break;
        case 15:
            Serial.println("Error, Reset System");
            resetSystem(-1);
            led(2);
            delay(3000);
            break;
        case 16:
            led(1);
            Serial.println("Reset System");
            resetSystem(-1);
            break;
        default:
            led(0);
            break;
    }

}