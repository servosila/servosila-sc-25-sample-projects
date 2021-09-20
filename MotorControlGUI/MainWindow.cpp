/////////////////////////////////////////////////////////////////////////////
//
//  This sample source code comes with Servosila SC-25C Brushless Motor Controllers.
//      OS: Windows or Linux
//      Interface to SC-25: SLCAN via USB virtual serial port
//
//  The code is provided "AS IS" without any kind of guarantees or warranties.
//  Use it at your own risk.
//
//  The code is free for everyone to use, modify or redistribute.
//
//  www.servosila.com
//
/////////////////////////////////////////////////////////////////////////////

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QSerialPortInfo>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_p_main_loop_timer(new QTimer(this))     //creating "Main Loop" timer object
{
    ui->setupUi(this);

    //initializing the "Main Loop" timer
    connect(m_p_main_loop_timer, &QTimer::timeout, this, &MainWindow::main_loop);
    //m_p_main_loop_timer->setTimerType(Qt::PreciseTimer);
    m_p_main_loop_timer->setTimerType(Qt::CoarseTimer);
    //This timer's period defines how often commands are going to be sent to the controller,
    //...and how often telemetry messages are read out.
    //Do not send commands to the contoller too often, as the controller's CPU might be overloaded by the incoming USB traffic.
    m_p_main_loop_timer->start(100);    //ms;

    //populating a list of serial ports
    on_pushButtonRefresh_clicked();

    //enable or disable GUI controls
    manage_gui();

    //resetting the runtime state parameters
    m_node_id = 1;
    m_is_sending_ongoing = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

//This function is periodically called using QT Timer mechanism.
//  The function serves the purpose of the "Main Loop" of a typical control system.
void MainWindow::main_loop()
{
    if(m_serial_port.isOpen())
    {
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Step 1: Reading out Telemetry
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        char symbol = 0;
        //Reading out all available symbols one by one from the virtual serial port
        while(true)
        {
            //reading out a single symbol if it is available
            const ssize_t nread = m_serial_port.read(&symbol, sizeof(symbol));
            if (nread > 0)
            {
                //feeding the symbol to the SLCAN decoder object
                //... the method returns true if a complete SLCAN message has been received
                const bool is_message_received = m_decoder.process_symbol(symbol);

                if(is_message_received) //a complete SLCAN message has been received
                {
                    //extracting CAN ID from the decoder object
                    const uint32_t CAN_ID  = m_decoder.get_can_id();
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
                            memcpy(&fault_bits, &(m_decoder.get_payload()[0]), sizeof(fault_bits));   //extracting INT16

                            //decoding Udc voltage (FLOAT16, position in Payload: 2)
                            int16_t Udc_float16 = 0;    //ATTENTION: FLOAT16 is transmitted as INT16
                            memcpy(&Udc_float16, &(m_decoder.get_payload()[2]), sizeof(Udc_float16));
                            const float Udc = servosila::decode_float16(Udc_float16);   //converting INT16->FLOAT32

                            //decoding Speed in Hz, electrical (FLOAT32, position in Payload: 4)
                            float SPEED = 0.0;
                            memcpy(&SPEED, &(m_decoder.get_payload()[4]), sizeof(SPEED)); //extracting FLOAT32

                            //filtering out telemetry related to the Node ID of interest
                            if(NODE_ID == m_node_id)    //remove this line if you want to receive telemetry from all controllers on CAN network
                            {
                                process_telemetry(fault_bits, Udc, SPEED);  //calling an application-specific routine to display telemetry once the telemetry message has been decoded
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

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Step 2: Sending out commands (periodically)
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        if(m_is_sending_ongoing)    //this flag is enabled or disabled by the "Start/Stop" button or by "Reset" button
        {
            //reading out speed target from the GUI
            const float speed_target = ui->doubleSpinBoxSpeed->value();

            //calling a procedure that encodes and sends out an Electronic Speed Control command
            send_speed_command(speed_target);
        }
    }
}

//a helper function that send out a Electronic Speed Control command
void MainWindow::send_speed_command(float speed_target)
{
    const uint32_t COB_ID       = 0x200;    //this value comes from Servosila Device Reference document, section related to "Electronic Speed Control" command
    const uint8_t  COMMAND_CODE = 0x20;     //this value comes from Servosila Device Reference document, section related to "Electronic Speed Control" command

    //an SLCAN message buffer: an array of chars
    char message[27];
    //a CAN Payload array (8bytes)
    uint8_t payload[8];

    //zero out all 8 bytes in the payload before filling out with new data
    memset(&payload, 0, sizeof(payload));

    //setting Command Code in Payload
    payload[0] = COMMAND_CODE;          //the very first byte in payload of a command message is the command code.

    //setting Speed parameter in Payload
    // That the parameter is a FLOAT32 with position 4 comes from Servosila Device Reference document, section related to "Electronic Speed Control" command.
    memcpy(&(payload[4]), &(speed_target), sizeof(speed_target));

    //encoding an SLCAN text message
    //...the routine is implemented in slcan-encoder.h
    //...the inputs are Node ID, COB ID, and Payload.
    const size_t message_size = servosila::slcan_encode_11bit(m_node_id+COB_ID, payload, 8, message);

    //writing the SLCAN text message to the virtual serial port
    m_serial_port.write(&(message[0]), message_size);
}

//a helper function that send out a STOP command
void MainWindow::send_stop_command()
{
    const uint32_t COB_ID       = 0x200;    //this value comes from Servosila Device Reference document
    const uint8_t  COMMAND_CODE = 0x04;     //this value comes from Servosila Device Reference document

    //an SLCAN message buffer: an array of chars
    char message[27];
    //a CAN Payload array (8bytes)
    uint8_t payload[8];

    //zero out all 8 bytes in the payload before filling out with new data
    memset(&payload, 0, sizeof(payload));

    //setting Command Code in Payload
    payload[0] = COMMAND_CODE;          //the very first byte in payload of a command message is the command code.

    //encoding an SLCAN text message
    //...the routine is implemented in slcan-encoder.h
    //...the inputs are Node ID, COB ID, and Payload.
    const size_t message_size = servosila::slcan_encode_11bit(m_node_id+COB_ID, payload, 8, message);

    //writing the SLCAN text message to the virtual serial port
    m_serial_port.write(&(message[0]), message_size);
}

//a helper function that send out a RESET command
void MainWindow::send_reset_command()
{
    const uint32_t COB_ID       = 0x200;    //this value comes from Servosila Device Reference document
    const uint8_t  COMMAND_CODE = 0x01;     //this value comes from Servosila Device Reference document

    //an SLCAN message buffer: an array of chars
    char message[27];
    //a CAN Payload array (8bytes)
    uint8_t payload[8];

    //zero out all 8 bytes in the payload before filling out with new data
    memset(&payload, 0, sizeof(payload));

    //setting Command Code in Payload
    payload[0] = COMMAND_CODE;          //the very first byte in payload of a command message is the command code.

    //encoding an SLCAN text message
    //...the routine is implemented in slcan-encoder.h
    //...the inputs are Node ID, COB ID, and Payload.
    const size_t message_size = servosila::slcan_encode_11bit(m_node_id+COB_ID, payload, 8, message);

    //writing the SLCAN text message to the virtual serial port
    m_serial_port.write(&(message[0]), message_size);
}

//This routine is called from the main_loop() whenever a telemetry message is received
//  The routine just displays the telemetry on the GUI.
void MainWindow::process_telemetry(uint16_t fault_bits, float Udc, float speed)
{
    if(fabsf(speed) < 0.0001) speed = 0.0;    //cosmetics for GUI: removing very small speed readings
    //displaying the telemetry data on the GUI
    ui->lcdNumberSpeed->display(speed);
    ui->lcdNumberInputVoltage->display(Udc);
    ui->labelFaultBits->setText(QString::number(fault_bits));
}

//This routine disables or enables GUI controls depending of the status of Serial Port connnection
void MainWindow::manage_gui()
{
    //a simple GUI state machine
    if(m_serial_port.isOpen())
    {
        ui->pushButtonConnect->setText(tr("Disconnect"));
        ui->pushButtonRefresh->setEnabled(false);
        ui->comboBoxSerialPorts->setEnabled(false);
        ui->spinBoxNodeID->setEnabled(false);
        ui->groupBoxTelemetry->setEnabled(true);
        ui->groupBoxCommand->setEnabled(true);
        ui->frameStateManagement->setEnabled(true);
    }
    else
    {
        ui->pushButtonConnect->setText(tr("Connect"));
        ui->pushButtonRefresh->setEnabled(true);
        ui->comboBoxSerialPorts->setEnabled(true);
        ui->spinBoxNodeID->setEnabled(true);
        ui->groupBoxTelemetry->setEnabled(false);
        ui->groupBoxCommand->setEnabled(false);
        ui->frameStateManagement->setEnabled(false);

        ui->pushButtonStart->setText(tr("Start"));
        m_is_sending_ongoing = false;
    }
}

//this routine populates a list of serial ports in a combobox GUI
void MainWindow::on_pushButtonRefresh_clicked()
{
    //Resetting the combobox of Interfaces
    ui->comboBoxSerialPorts->clear();

    //Serial: populating the combobox of interfaces with a list of available COM ports
    const QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(QSerialPortInfo info, ports)
    {   //inserting a line into the combobox
        ui->comboBoxSerialPorts->addItem(info.portName());
    }

    //selecting the last item (latest COM port)
    const size_t count = ui->comboBoxSerialPorts->count();
    if(count > 1) ui->comboBoxSerialPorts->setCurrentIndex(count - 1);
}

//This routine connects to or disconnects the program from Serial Port
void MainWindow::on_pushButtonConnect_clicked()
{
    //updating Node ID of the device we are going to control
    m_node_id = ui->spinBoxNodeID->value();

    //connecting to or disconnecting from the serial port
    if(!m_serial_port.isOpen())
    {
        const QString serial_port_name = ui->comboBoxSerialPorts->currentText();

        //connecting to a selected serial port
        m_serial_port.setPortName(serial_port_name);
        const bool result = m_serial_port.open(QIODevice::ReadWrite);
        if(!result)
        {
            QMessageBox::information(this, tr("Serial Port"), tr("Cannot open the serial port."), QMessageBox::Ok);
        }
    }
    else
    {
        m_serial_port.close();
        m_is_sending_ongoing = false;
    }

    //enable or disable GUI controls
    manage_gui();
}

//this routine raises or clears a flag (m_is_sending_ongoing) that controls whether or not the main loop periodically sends commands to the contoller
//In addition, this routine sends our a STOP command to the controller (once) when needed.
void MainWindow::on_pushButtonStart_clicked()
{
    if(m_is_sending_ongoing)
    {
        ui->pushButtonStart->setText("Start");
        m_is_sending_ongoing = false;

        //calling a procedure that sends out a STOP command to the controller
        send_stop_command();
    }
    else
    {
        ui->pushButtonStart->setText("Stop");
        m_is_sending_ongoing = true;
    }
}

/*
The RESET command clears "Fault Bits" latches, powers off the motor, resets the inverter circuitry, and resets the Work Zone
position. Use this command to clear fault flags whenever Fault Bits telemetry indicates a fault, to reset servo position
within the work zone, or as a "panic button" to power off the motor in an emergency.
Whenever a fault is detected, the controller powers off the motor, raises one or more "Fault Bits" flags, and starts
waiting for a "Reset" command to come from a parent control system.
*/
void MainWindow::on_pushButtonReset_clicked()
{
    //sending out a RESET command to the controller
    send_reset_command();

    //stopping periodic sending of the ESC command
    ui->pushButtonStart->setText(tr("Start"));
    m_is_sending_ongoing = false;
}
