#ifndef URDUINO_HH
# define URDUINO_HH

#include <urbi/uobject.hh>
#include "arduino.hh"

#define BEGIN_TRANSMISSION (uint8_t) '\001'
#define END_TRANSMISSION   (uint8_t) '\004'

class Urduino: public urbi::UObject, public Arduino
{
	public:
		Urduino(const std::string &n);

		int init(std::string serialport, int baudrate);

		std::string readString();

		/*!
		 * \brief Send command to the Arduino
		 * \param type  Clamp : 0 and Movement : 1
		 * \param id    type : 0 -> { close : 0 and open : 1 }
		 *              and type : 1 -> { translate : 0 and rotate : 1 }
		 * \param value if type : 1 then angle or distance
		 */
		void writeCommand( int type, int id, int value );

		void rotate( ufloat rad );

		void translate( ufloat meters );

		void openClamp();

		void closeClamp();
};

#endif
