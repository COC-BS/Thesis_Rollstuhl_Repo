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

//********************************Setup Loop*********************************//

#define CANEn 22  // Externer Pin für CAN Shield Enable, Slave Select


void setup() {
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
}

//********************************Main Loop*********************************//

void loop() //Loop darf nicht länger als 200ms gehen, sonst automatischer Stopp der Steuerung
{
    tCAN message;

    int forwardSpeed = 0; // Gescchwindigkeit (0 - 100)
    int turnRate = 0; // + -> rechts Kurve  |
    // - -> links Kurve | 0 -> Geradeaus || 100 = L vorwärtas, R rückwärts, 50 nur ein Rad in Betrieb

    //====== Write on CAN-Bus =========================================


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


    //====== Read from CAN-Bus =========================================

        int joystickDirection;
        int joystickSpeed;

        if (mcp2515_get_message(&message))
        {
            //Joystick-Data
            /*
            if(message.id == 0x081) //Filter ID
            {
                joystickDirection = message.data[0];
                joystickSpeed = message.data[1];

                Serial.println("Joystick:   Direction: " + String(joystickDirection) + ";  Speed: " + String(joystickSpeed));
            }
             */
        }

        /* Motorenencoder auslesen
        if (mcp2515_get_message(&message))
        {
            if(message.id == 0x7FF) //Filter ID
            {
                //Get left and right Motorspeed
                float currSpeedLeft = (message.data[4] + message.data[5] * 256) * 0.345 * PI / (60 * 23);
                float currSpeedRight = (message.data[6] + message.data[7] * 256) * 0.345 * PI / (60 * 23);
                Serial.println("Encoder:   Left: " + String(currSpeedLeft) + ";  Right: " + String(currSpeedRight));
            }
        }
        */

}