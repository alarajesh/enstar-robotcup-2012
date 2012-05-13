#include "Urduino.hh"

#include <cmath>

Urduino::Urduino(const std::string &n) :
	urbi::UObject(n),
	Arduino()
{
	UBindFunction( Urduino, init );
}

int Urduino::init(std::string serialport, int baudrate)
{
	//UBindFunction( Urduino, writeString );
	//UBindFunction( Urduino, writeCommand );
	UBindFunction( Urduino, openClamp );
	UBindFunction( Urduino, closeClamp );
	UBindFunction( Urduino, rotate );
	UBindFunction( Urduino, translate );
	UBindFunction( Urduino, readString );

	return Arduino::init(serialport, baudrate);
}

void* Urduino::listenPort( void* )
{
	std::string current_msg;
	fd_set read_set;
	char buffer[1];

	while ( true )
	{
		FD_SET( Arduino::mFd, &read_set );
		select( Arduino::mFd + 1, &read_set, NULL, NULL, NULL );

		if ( FD_ISSET( Arduino::mFd, &read_set ) )
		{
			// read the next char
			int n = read( Arduino::mFd, buffer, 1);

			if ( n <= 0 ) continue;

			if ( buffer[0] != '\n' )
			{
				current_msg.append( 1, buffer[0] );
			}
			else
			{
				std::cout << current_msg << std::endl;
				current_msg.clear();
			}
		}
	}
}

void Urduino::writeCommand( int type, int id, int value )
{
	uint8_t* tab = (uint8_t*) &value;

	writeByte( BEGIN_TRANSMISSION );
	writeByte( (uint8_t) type );
	writeByte( (uint8_t) id );
	writeByte( tab[0] );
	writeByte( tab[1] );
	writeByte( tab[2] );
	writeByte( tab[3] );
	writeByte( END_TRANSMISSION );
}

void Urduino::rotate( ufloat rad )
{
	int value = (int) ( (rad * 458 * 2) / M_PI );

	writeCommand( 1, 1, value );
}

void Urduino::translate( ufloat meters )
{
	int value = (int) ( (meters * 700) / 0.3 );

	writeCommand( 1, 0, value );
}

void Urduino::openClamp()
{
	writeCommand( 0, 1, 0 );
}

void Urduino::closeClamp()
{
	writeCommand( 0, 0, 0 );
}

UStart(Urduino);