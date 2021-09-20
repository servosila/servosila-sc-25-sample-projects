/////////////////////////////////////////////////////////////////////////////////////////////////
//
//  This sample source code comes with Servosila SC-25C Brushless Motor Controllers.
//
//  This example sends an Electronic Speed Control (ESC) command in a loop to a controller.
//      OS: Linux,
//      Interface: SLCAN text protocol via virtual servial port.
//
//  The code is provided "AS IS" without any kind of guarantees or warranties.
//  Use it at your own risk.
//
//  The code is free for everyone to use, modify or redistribute.
//
//  www.servosila.com
//
/////////////////////////////////////////////////////////////////////////////////////////////////

#include "../servosila-common/slcan-encoder.h"  //SLCAN encoder function
#include <fstream>                              //file stream output
#include <string.h>                             //memcpy(), memset()
#include <stdint.h>                             //standard integer types
#include <chrono>                               //sleep(), C++11
#include <thread>                               //sleep(), C++11

int main()
{    
    //a standard C++ stream object for writing to a virtual serial port on Linux
    std::ofstream device;

    //opening virtual serial port...
    //...check that the file name is correct...
    device.open ("/dev/ttyACM0");           //if this fails on Linux: sudo usermod -G dialout $USER

    //an SLCAN message buffer: an array of chars
    char message[27];
    //a CAN Payload array (8bytes)
    uint8_t payload[8];

    const uint32_t NODE_ID      = 5;        //this is a unique Node ID of the device. Change this to match your device.
    const uint32_t COB_ID       = 0x200;    //this value comes from Servosila Device Reference document, section related to "Electronic Speed Control" command
    const uint8_t  COMMAND_CODE = 0x20;     //this value comes from Servosila Device Reference document, section related to "Electronic Speed Control" command
    const float    SPEED        = 100.0;    //Hz (electrical), this is a target speed that needs to be sent to the controller. A constant in this example, but normally this is dynamically computed.

    //Main Loop
    for(size_t i=0; i<100; i++)     //this should normally be a while(true) loop
    {
        //zero out all 8 bytes in the payload before filling out with new data
        memset(&payload, 0, sizeof(payload));

        //setting Command Code in Payload
        payload[0] = COMMAND_CODE;          //the very first byte in payload of a command message is the command code.

        //setting Speed parameter in Payload
        // That the parameter is a FLOAT32 with position 4 comes from Servosila Device Reference document, section related to "Electronic Speed Control" command.
        memcpy(&(payload[4]), &(SPEED), sizeof(SPEED));

        //encoding an SLCAN text message
        //...the routine is implemented in slcan-encoder.h
        //...the inputs are Node ID, COB ID, and Payload.
        const size_t message_size = servosila::slcan_encode_11bit(NODE_ID+COB_ID, payload, 8, message);

        //writing the SLCAN text message to the virtual serial port
        device.write(&(message[0]), message_size);
        device.flush();                     //this is needed before the sleep() function

        //TODO: read out and process telemetry here (see a different example)

        //this is just a portable way to sleep() in the main loop...
        //...this method requires C++11
        std::this_thread::sleep_for(std::chrono::milliseconds(200));    //200ms=5Hz; sending the command 5 times a second; do not send commands too often as the controller wastes CPU cycles on this, it could otherwise use the cycles to better run the motor.
    }

    //closing the virtual serial port
    device.close();

    return 0;
}
