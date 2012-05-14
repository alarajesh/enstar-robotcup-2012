#ifndef POSSERVO_HH
# define POSSERVO_HH

#include <urbi/uobject.hh>
#include <math/quaternion.hpp>
#include <pthread.h>

#include "Cemaphore.hh"

#define BEGIN_TRANSMISSION (uint8_t) '\001'
#define END_TRANSMISSION   (uint8_t) '\004'

typedef boost::math::quaternion< ufloat > Quaternion;

class PosServo: public urbi::UObject
{
	public:
		PosServo( const std::string &n );

		int init();
		void getNextMovement();
		void onNewMarkerPositions( urbi::UVar& markers );

	private:
		urbi::UVar markerPositions; // UList(UList(ufloat)) // input
		urbi::UVar movement;        // UList(ufloat) xyzt   // output
};

#endif
