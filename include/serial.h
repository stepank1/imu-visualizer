#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

class Serial
{
    int  serial_port;
public:
    bool openSerial();
    bool configureSerial();
    bool readLine(char*, size_t);
    void closeSerial();
};
