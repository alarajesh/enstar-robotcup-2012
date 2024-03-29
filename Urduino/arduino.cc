/*
 * Arduino-serial
 * --------------
 *
 * Class c++ to communicate with arduino
 *
 */

#include "arduino.hh"

int Arduino::writeByte( uint8_t b)
{
	int n = write(mFd,&b,1);
	if( n!=1)
		return -1;
	return 0;
}

int Arduino::writeString(const std::string str)
{
	int len = strlen(str.c_str());
	int n = write(mFd, str.c_str(), len);
	if( n!=len )
		return -1;
	return 0;
}

int Arduino::readUntil(std::string &buf, char until)
{
	buf.clear();
	char b[1];
	int i=0;
	while (true)
	{
		int n = read(mFd, b, 1);  // read a char at a time

		if ( n==-1 ) return -1;    // couldn't read

		if ( n==0 )
		{
			usleep( 10 * 1000 ); // wait 10 msec try again
			continue;
		}

		if ( b[0] != until )
		{
			buf.append(1,b[0]);
			i++;
		}
		else
		{
			break;
		}
	}

	return 0;
}

// takes the string name of the serial port (e.g. "/dev/ttyACM0")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int Arduino::init(const std::string serialport, const int baud)
{
	struct termios toptions;

	//fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
	//        serialport,baud);

	mFd = open(serialport.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (mFd == -1)  {
		perror("init_serialport: Unable to open port ");
		return -1;
	}

	if (tcgetattr(mFd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
		return -1;
	}
	speed_t brate = baud; // let you override switch below if needed
	switch(baud) {
		case 4800:   brate=B4800;   break;
		case 9600:   brate=B9600;   break;
#ifdef B14400
		case 14400:  brate=B14400;  break;
#endif
		case 19200:  brate=B19200;  break;
#ifdef B28800
		case 28800:  brate=B28800;  break;
#endif
		case 38400:  brate=B38400;  break;
		case 57600:  brate=B57600;  break;
		case 115200: brate=B115200; break;
	}
	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 20;

	if( tcsetattr(mFd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}

	return 0;
}


