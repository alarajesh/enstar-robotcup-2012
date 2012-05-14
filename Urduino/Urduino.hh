#ifndef URDUINO_HH
# define URDUINO_HH

#include <urbi/uobject.hh>
#include <pthread.h>
#include "arduino.hh"
#include "Functor.hh"
#include "Cemaphore.hh"

#define BEGIN_TRANSMISSION (uint8_t) '\001'
#define END_TRANSMISSION   (uint8_t) '\004'

class Urduino: public urbi::UObject, public Arduino
{
	public:
		Urduino( const std::string &n );

		int init( std::string serialport, int baudrate );

		void rotate( ufloat rad );

		void translate( ufloat meters );

		void openClamp();

		void closeClamp();

		urbi::UList getDPosition( bool resetDiff=true );

	private:
		/*!
		 * brief Loop which listen the serial port and interpret the command received.
		 */
		void* listenPort( void* unused );

		/*!
		 * \brief Send command to the Arduino
		 * \param type  Clamp : 0 and Movement : 1
		 * \param id    type : 0 -> { close : 0 and open : 1 }
		 *              and type : 1 -> { translate : 0 and rotate : 1 }
		 * \param value if type : 1 then angle or distance
		 */
		void writeCommand( int type, int id, int value );

		// diffential of position and rotation
		Cemaphore < std::complex< float > > dposition;
		Cemaphore < float > dangle;

		// Cumulate position and rotation
		Cemaphore < std::complex< float > > position;
		Cemaphore < float > angle;

		// arduino listening stuff
		Functor< Urduino > listeningFunctor;
		pthread_t listeningThread;
};

#endif
