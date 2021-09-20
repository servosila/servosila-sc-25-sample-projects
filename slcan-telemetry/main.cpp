///////////////////////////////////////////////////////////////////////////////////////////////
//
//  This sample source code comes with Servosila SC-25C Brushless Motor Controllers.
//
//  This example receives Telemetry messages in a loop and prints out the telemetry data.
//      OS: Linux,
//      Interface to controllers: SLCAN text protocol via USB virtual servial port.
//
//  The code is provided "AS IS" without any kind of guarantees or warranties.
//  Use it at your own risk.
//
//  The code is free for everyone to use, modify or redistribute.
//
//  www.servosila.com
//
///////////////////////////////////////////////////////////////////////////////////////////////

#include "../servosila-common/slcan-decoder.h"      //SLCAN decoder class
#include "../servosila-common/canopen-decoder.h"    //CANopen decoding functions
#include <fstream>                                  //file stream input
#include <iostream>                                 //console output
#include <string.h>                                 //memcpy(), memset()
#include <stdint.h>                                 //standard integer types
#include <chrono>                                   //sleep(), C++11
#include <thread>                                   //sleep(), C++11

int main()
{
    //a standard C++ stream object for reading from a virtual serial port on Linux
    std::fstream device;

    //opening virtual serial port...
    //...check that the file name is correct...
    device.open ("/dev/ttyACM0");   //if this fails on Linux: sudo usermod -G dialout $USER

    //this is a SLCAN decoder object
    //...the class is defined in slcan-decoder.h
    servosila::slcan_decoder decoder;

    //Main Loop
    while(true)
    {
        char symbol = 0;
        //Reading out all available symbols one by one from the virtual serial port
        while(true)
        {
            //reading out a single symbol if it is available
            const ssize_t nread = device.readsome(&symbol, sizeof(symbol));
            if (nread > 0)
            {
                //feeding the symbol to the SLCAN decoder object
                //... the method returns true if a complete SLCAN message has been received
                const bool is_message_received = decoder.process_symbol(symbol);

                if(is_message_received) //a complete SLCAN message has been received
                {
                    //extracting CAN ID from the decoder object
                    const uint32_t CAN_ID  = decoder.get_can_id();
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
                            memcpy(&fault_bits, &(decoder.get_payload()[0]), sizeof(fault_bits));   //extracting INT16

                            //decoding Udc voltage (FLOAT16, position in Payload: 2)
                            int16_t Udc_float16 = 0;    //ATTENTION: FLOAT16 is transmitted as INT16
                            memcpy(&Udc_float16, &(decoder.get_payload()[2]), sizeof(Udc_float16));
                            const float Udc = servosila::decode_float16(Udc_float16);   //converting INT16->FLOAT32

                            //decoding Speed in Hz, electrical (FLOAT32, position in Payload: 4)
                            float SPEED = 0.0;
                            memcpy(&SPEED, &(decoder.get_payload()[4]), sizeof(SPEED)); //extracting FLOAT32

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
            }
            else
            {   //no symbols left to be read out from the virtual serial port
                //...just breaking from the loop to work on something else
                break;
            }
        } //while() for reading out symbols

        //TODO: send out commands to controllers here (see a different example)

        //this is just a portable way to sleep() in the main loop...
        //...this method requires C++11
        std::this_thread::sleep_for(std::chrono::milliseconds(200));    //200ms=5Hz; reading out telemetry 5 times a second

    } //while() of the main loop

    device.close();

    return 0;
}
