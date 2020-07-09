/****************************************************************************
CAN Write Demo for the SparkFun CAN Bus Shield.

Written by Stephen McCoy.
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA.

Distributed as-is; no warranty is given.
*************************************************************************/

#include <Arduino.h>

#include <mcp2515/mcp2515.h>
#include <mcp2515/mcp2515_defs.h>
#include <Canbus/Canbus.h>
#include <PID_v1/PID_v1.h>

//********************************Setup Loop*********************************//

#define CANEn 22  // Externer Pin für CAN Shield Enable, Slave Select

//PID Variabeln
double calcAngle = 90;
double turnedAngle = 0; //Winkel der durch Encoder ermittelt wurde
double Setpoint; //Soll-Winkel
double Input; //Differenz zwischen soll und ist winkel
double Output; //IST-Winkel, für Motorenansteuerung nutzen
//PID-Parameter
double Kp=20, Ki=10, Kd=10;
//PID Instanz erstellen
PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int currMillis = 0;
int oldMillis = -1;


void readCAN (tCAN message) {

    int joystickDirection;
    int joystickSpeed;

    if (mcp2515_get_message(&message))
    {
        /*
        //Read Joystick-Data
        if(message.id == 0x081) //Filter ID
        {
            joystickDirection = message.data[0];
            joystickSpeed = message.data[1];

            Serial.println("Joystick:   Direction: " + String(joystickDirection) + ";  Speed: " + String(joystickSpeed));
        }
         */

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

int calcMotorInput (double angle) {


}


void setup() {
    //CAN-Initialisierung
    Serial.begin(9600);
    pinMode(CANEn,OUTPUT);
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
    Setpoint = calcAngle;
    Output = 0;
    motorPID.SetMode(AUTOMATIC); //Turn PID on
    motorPID.SetTunings(Kp,Ki,Kd); //Adjust PID values
}


void loop() //Loop darf nicht länger als 200ms gehen, sonst automatischer Stopp der Steuerung
{
    tCAN message;

    int forwardSpeed = 0; // Gescchwindigkeit (0 - 100)
    int turnRate = 0; // + -> rechts Kurve  | - -> links Kurve | 0 -> Geradeaus || 100 = L vorwärtas, R rückwärts, 50 nur ein Rad in Betrieb

    readCAN(message);
    //writeCAN(message,forwardSpeed,turnRate);

    //=========PID-Verarbeitung===============
    Setpoint = calcAngle;
    Input = turnedAngle;

    motorPID.Compute(); //PID berechnung

    Serial.println("Output PID: " + String(Output) + "  Turned Angle: " + String(turnedAngle));

    writeCAN(message,0,(Output*-1)); //Gibt Motorensteuerung anhand Output PID

    //Motoren ansteuern

}