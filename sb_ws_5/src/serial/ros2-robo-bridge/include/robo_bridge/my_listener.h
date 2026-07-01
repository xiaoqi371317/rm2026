#pragma once

#include <vector>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

using namespace itas109;

class MyListener : public CSerialPortListener
{
public:
    MyListener(CSerialPort *sp)
        : p_sp(sp){};

    void onReadEvent(const char *portName, unsigned int readBufferLen);

private:
    CSerialPort *p_sp;
};
