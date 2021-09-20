///////////////////////////////////////////////////////////////////////////////////////////////
//
//  This sample source code comes with Servosila SC-25C Brushless Motor Controllers.
//  This example receives Telemetry messages in a loop and prints out telemetry data.
//      OS: Linux,
//      Interface: Linux SocketCAN API
//
//  The code is provided "AS IS" without any kind of guarantees or warranties.
//  Use it at your own risk.
//
//  The code is free for everyone to use, modify or redistribute.
//
//  www.servosila.com
//
///////////////////////////////////////////////////////////////////////////////////////////////

#include "../servosila-common/canbus.h"             //SocketCAN encapsulation
#include "../servosila-common/canopen-decoder.h"    //CANopen helper functions
#include <iostream>                                 //console output
#include <string.h>                                 //memcpy(), memset()
#include <stdint.h>                                 //standard integer types
#include <chrono>                                   //sleep(), C++11
#include <thread>                                   //sleep(), C++11

int main()
{
    //An object that encapsulates Linux SocketCAN API
    //...An alternative is to use QT's CANbus classes.
    servosila::canbus canbus;

    //starting up SocketCAN encapsulation object
    canbus.startup("can0");     //check the network name, it could be different in your system

    if(canbus.is_connected())
    {
        //Main Loop
        while(true)
        {
            uint32_t CAN_ID;
            uint8_t payload[8];
            uint8_t nbytes_received;

            //reading out a CAN frame
            const bool is_message_received = canbus.receive(&payload, sizeof(payload), nbytes_received, CAN_ID);

            if(is_message_received)
            {
                //using helper functions to split CAN ID into NODE ID and COB ID
                const uint32_t NODE_ID = servosila::extract_node_id_from_can_id(CAN_ID);    //this ID tells what of the controllers on CAN network sent the telemetry message
                const uint32_t COB_ID  = servosila::extract_cob_id_from_can_id (CAN_ID);    //this ID tells how to decode the message (format). Refer to Servosila Device Reference document for telemetry message formats by their COB IDs.

                //applying different decoding logic (format) depending on what telemetry message has been received
                switch(COB_ID)
                {
                    case 0x180:
                    {
                        //decoding Fault Bits (UINT16, position in Payload: 0)
                        uint16_t fault_bits = 0;
                        memcpy(&fault_bits, &(payload[0]), sizeof(fault_bits));   //extracting INT16

                        //decoding Udc voltage (FLOAT16, position in Payload: 2)
                        int16_t Udc_float16 = 0;   //ATTENTION: FLOAT16 is transmitted as INT16
                        memcpy(&Udc_float16, &(payload[2]), sizeof(Udc_float16));
                        const float Udc = servosila::decode_float16(Udc_float16);   //converting INT16->FLOAT32

                        //decoding Speed in Hz, electrical (FLOAT32, position in Payload: 4)
                        float SPEED = 0.0;
                        memcpy(&SPEED, &(payload[4]), sizeof(SPEED)); //extracting FLOAT32

                        //printing out the data for demo purposes
                        std::cout<<"Node ID: "<<NODE_ID<<" Fault Bits: "<<fault_bits<<" "<<Udc<<" V DC Speed: "<<SPEED<<" Hz"<<std::endl;
                        std::cout.flush();

                        //Handiling faults
                        if(fault_bits != 0)
                        {   //FAULT REPORTED BY THE DEVICE
                            //...the controller keeps the motor de-energized until a "Reset" command comes.
                            //TODO: send "Reset" command here once the fault has been rectified...
                        }

                        break;
                    }
                    case 0x280:
                    case 0x380:
                    case 0x480:
                    {
                        //TODO: Add other telemetry handlers here
                        //...the formats are defined in Servosila Device Reference document for your device.
                        //The processing principle is the same as for the 0x180 telemetry message.
                        break;
                    }
                }
            }

            //TODO: send out commands to controllers here (see a different example)

            //this is just a portable way to sleep() in the main loop...
            //...this method requires C++11
            std::this_thread::sleep_for(std::chrono::milliseconds(200));    //200ms=5Hz; reading out telemetry 5 times a second

        } //while() of the main loop

        //shutting down SocketCAN encapsulation object
        canbus.shutdown();
    }

    return 0;
}
