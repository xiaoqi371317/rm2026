#include "robo_bridge/my_listener.h"
#include "robo_bridge/robo_bridge.h"
#include "robo_bridge/OffboardLink.h"

extern RoboBridge bridge;
unsigned char data[2048];

void MyListener::onReadEvent(const char *portName, unsigned int readBufferLen)
{
    if (readBufferLen > 0)
    {
        if (data)
        {
            // read
            const unsigned int read_len = readBufferLen > sizeof(data) ? static_cast<unsigned int>(sizeof(data)) : readBufferLen;
            int recLen = p_sp->readData(data, static_cast<int>(read_len));

            if (recLen > 0)
            {
                for(int i = 0; i < recLen; i ++)
                {
                    // ROS_INFO("%d 0x%02x", recLen, data[i]);
                    bridge.process_byte(data[i]);
                }
            }
        }
    }
};
