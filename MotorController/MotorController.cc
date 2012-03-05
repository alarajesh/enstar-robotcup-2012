#include "MotorController.hh"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

MotorController::MotorController(const std::string& name) : /*{{{*/
	urbi::UObject(name)
{
	UBindFunction(MotorController, init);
} /*}}}*/

MotorController::~MotorController() /*{{{*/
{
	close(fd_serial_port);
} /*}}}*/

void MotorController::init() /*{{{*/
{
	UBindVar(MotorController, inputMotor1Speed);
	UBindVar(MotorController, inputMotor2Speed);
	UBindVar(MotorController, outputMotor1Current);
	UBindVar(MotorController, outputMotor2Current);
	UBindVar(MotorController, outputMotor1Encoder);
	UBindVar(MotorController, outputMotor2Encoder);

	UBindFunction(MotorController, testWrite);
	UBindFunction(MotorController, testRead);

	// open serial port
	openPort();
} /*}}}*/

bool MotorController::openPort() /*{{{*/
{
	fd_serial_port = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd_serial_port == -1) /*{{{*/
	{
		// on failure
		std::cerr << "open_port: Unable to open /dev/ttyS0 - "
			<< std::endl;
		return false;
	} /*}}}*/

	// on success
	std::cout << "serial port is open" << std::endl;

	// set the file status flag to Read/Write
	fcntl(fd_serial_port, F_SETFL, NULL);

	// Get the current serial_port_options for the port...
	tcgetattr(fd_serial_port, &serial_port_options);

	// Set the baud rates to 9600...
	cfsetispeed(&serial_port_options, B9600);
	cfsetospeed(&serial_port_options, B9600);

	// Enable the receiver and set local mode...
	serial_port_options.c_cflag |= (CLOCAL | CREAD);

	// Set the new serial_port_options for the port...
	tcsetattr(fd_serial_port, TCSANOW, &serial_port_options);

	return true;
} /*}}}*/

bool MotorController::testWrite() /*{{{*/
{
	if (write(fd_serial_port, "ATZ\r", 4) < 0) /*{{{*/
	{
		std::cerr << "write() of 4 bytes failed!" << std::endl;
		return false;
	} /*}}}*/

	return true;
} /*}}}*/

bool MotorController::testRead() /*{{{*/
{
	char buffer[200];

	if (read(fd_serial_port, buffer, 6))
		std::cout << buffer << std::endl;
	else
		std::cerr << "no message" << std::endl;
} /*}}}*/

UStart(MotorController);
