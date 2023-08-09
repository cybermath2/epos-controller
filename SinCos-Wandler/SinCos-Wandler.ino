// for CAN
#include "mcp_can.h"
#include <SPI.h>

// CAN TX Variables
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // 1Hz interval min without change (1sec)
const unsigned int inv2TX = 50;                                  // 20 Hz interval max on change (50ms)
byte data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Generic CAN data to send
#define CAN_ID 0x141
// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT (pin 2 for model "Andrä", 4 for model "Felix")
MCP_CAN CAN0(4);                                // Set CS (pin 4 for model "Andrä", 7 for model "Felix")

#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

float head = 0.0;
float headOld = 0.0;

void setup()
{
    Serial.begin(115200);

    // SOF_ENABLE

    // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");
    // Since we do not set NORMAL mode, we are in loopback mode by default.
    CAN0.setMode(MCP_NORMAL);
    pinMode(CAN0_INT, INPUT);

}

void loop()
{
    int gnd = analogRead(A0);
    int sinX = gnd - analogRead(A1);
    int cosX = gnd - analogRead(A2);
    float head = atan2(cosX, sinX) * RAD_TO_DEG;    // results in -180 to 180 degree

    // invert and round to 1 decimal
    head = - float(long(head * 10)) / 10.0;

    if (millis() - prevTX >= inv2TX){                // on change send at a 20Hz interval.
        if (headOld!=head)
        {
            prevTX -= invlTX;                           // fake last send to be older than invlTX
            headOld = head;
        }
    }

    if (millis() - prevTX >= invlTX){                // send alway at a one second interval.
        prevTX = millis();

        // print value
        Serial.print("heading = ");
        Serial.println(head);

        // Message Header
        data[0] = 0x2A;     // node-ID    (42)
        data[1] = 0x02;     // data type  (0x02=FLOAT)
        data[2] = 0x00;     // Service Code (unused for NOD)
        data[3]++;          // increase Message Code

        // Message Data
        byte *bp;                   // can point to any byte in RAM as a byte regardless of how it was declared
        bp = (byte *)( &head );     // takes the address of bigVar and casts it to a byte pointer
        memcpy(data + 4, bp, 4);

        // send CAN
        byte sndStat = CAN0.sendMsgBuf(CAN_ID, 8, data);

        if(sndStat == CAN_OK) {
            Serial.print("TX: ID = ");
            Serial.print(CAN_ID,HEX);
            Serial.print(", DATA =");
            for (size_t i = 0; i < 8; i++){
                Serial.print(" ");
                Serial.print(data[i], HEX);
            }
            Serial.println();
        } else {
            Serial.print("Error [");
            Serial.print(sndStat, HEX);
            Serial.println("] Sending Message...");
        }
    }

}
