#include "serial.h"

bool Serial::openSerial()
{
	serial_port = open("/dev/ttyACM0", O_RDWR);

	if (serial_port < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return false;
	}

	if (!configureSerial()) {
		printf("Error while configuring serial.");
		return false;
	}

	return true;
}

bool Serial::configureSerial()
{
	struct termios tty;

	if (tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return false;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// tty.c_lflag &= ~ICANON;
	tty.c_lflag |= ICANON | ISIG; // Canonical input 
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return false;
	}

	return true;
}

bool Serial::readLine(char *msg, size_t len)
{
	memset(msg, 0, len);

	int num_bytes = read(serial_port, msg, len);

	if (num_bytes < 0) {
		printf("Error reading: %s", strerror(errno));
		return false;
	}

	return true;
}

void Serial::closeSerial()
{
	close(serial_port);
}
