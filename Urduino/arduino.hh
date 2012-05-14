#ifndef ARDUINO_HH
# define ARDUINO_HH

#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <iostream>

class Arduino
{
	public:
		/*!
		 * \brief constructor
		 */
		Arduino() : mFd(-1) {}

		/*!
		 * \brief start communication
		 * \param serialport /dev/ttyACM*
		 * \param baudrate 9600
		 * \return the status of the connection -1 == failed
		 */
		int init(const std::string serialport, const int baudrate);

		/*!
		 * \brief write message
		 * \param message
		 * \return -1 if failed 0 else
		 */
		int writeString(const std::string message);

		/*!
		 * \brief read message until ending char is reached
		 * \param buf the string written to
		 * \param until the ending character
		 * \return -1 if failed 0 else
		 */
		int readUntil(std::string &buf, char until);

		/*!
		 * \brief write a byte to serial port
		 * \param b the byte to write
		 * \return -1 if failed 0 else
		 */
		int writeByte( uint8_t b);

	protected:
		int mFd;//file descriptor
};

#endif
