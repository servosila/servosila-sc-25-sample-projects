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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "../servosila-common/slcan-encoder.h"
#include "../servosila-common/slcan-decoder.h"
#include "../servosila-common/canopen-decoder.h"

#include <QMainWindow>
#include <QTimer>           //periodic ("Main Loop") timer
#include <QSerialPort>      //creoss-platform Serial Port API encapsulation

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonRefresh_clicked();
    void on_pushButtonConnect_clicked();
    void on_pushButtonStart_clicked();
    void on_pushButtonReset_clicked();

private:
    Ui::MainWindow *ui;
    //periodic timer ("Main Loop")
    QTimer* m_p_main_loop_timer;
    //cross-platform Serial Port object
    QSerialPort m_serial_port;
    //SLCAN decoder object
    servosila::slcan_decoder m_decoder;
    //A flag that tells that the user has initiated periodical sending of the command to the controller. The flag is updated by Start/Stop button.
    bool m_is_sending_ongoing;
    //Node ID of the controller. This attribute is updated from the GUI.
    uint32_t m_node_id;

private:
    void manage_gui();
    void main_loop();
    void process_telemetry(uint16_t fault_bits, float Udc, float speed);
    void send_speed_command(float speed_target);
    void send_stop_command();
    void send_reset_command();
};
#endif // MAINWINDOW_H
