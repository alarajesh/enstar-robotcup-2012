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

std::string Urduino::readString()
{
	std::string res;

	Arduino::readUntil( res, '\n' );

	return res;
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
